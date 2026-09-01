// SPDX-License-Identifier: GPL-2.0

//! Documentation and usage example of the macro can be found at `rust/kernel/io/register.rs`.

use proc_macro2::{
    Group,
    Literal,
    Span,
    TokenStream, //
};
use quote::{
    quote,
    quote_spanned, //
};
use syn::{
    bracketed,
    parenthesized,
    parse::Parse,
    spanned::Spanned,
    token,
    Attribute,
    Expr,
    Ident,
    Path,
    Result,
    Token,
    Type,
    Visibility, //
};

mod kw {
    syn::custom_keyword!(stride);
}

/// Definition of a register array.
///
/// Specify a size, and optionally a stride. Syntax is of form `[EXPR $(, stride = EXPR)?]`.
struct RegArrayDef {
    size: Expr,
    stride: Option<Expr>,
}

/// Offset of a register.
///
/// Can be either of form
/// * `@ offset` for fixed offset
/// * `=> alias` for alias of register `alias`.
/// * `=> alias[idx]` for alias of register array `alias`'s `idx`-th element.
enum RegOffset {
    /// Register is located at fixed address.
    Fixed { offset: Literal },
    /// Register is an alias of a fixed register.
    Alias { alias: Path },
    /// Register is an alias of an element of a register array.
    ElementAlias { alias: Path, idx: Expr },
}

/// Definition of a single register.
struct Reg {
    attrs: Vec<Attribute>,
    vis: Visibility,
    name: Ident,
    storage: Type,
    array: Option<RegArrayDef>,
    relative_base: Option<Path>,
    offset: RegOffset,
    bitfield_args: Group,
}

impl Parse for Reg {
    fn parse(input: syn::parse::ParseStream<'_>) -> Result<Self> {
        let attrs = input.call(Attribute::parse_outer)?;
        let vis = input.parse()?;
        let name = input.parse()?;
        let storage = {
            let content;
            parenthesized!(content in input);
            content.parse()?
        };
        let array = if input.peek(token::Bracket) {
            let content;
            bracketed!(content in input);
            let size = content.parse()?;
            let stride = if content.peek(Token![,]) {
                let _: Token![,] = content.parse()?;
                let _: kw::stride = content.parse()?;
                let _: Token![=] = content.parse()?;
                Some(content.parse()?)
            } else {
                None
            };
            Some(RegArrayDef { size, stride })
        } else {
            None
        };

        // Parse offset and the base it's relative to.
        let lh = input.lookahead1();
        let mut relative_base = None;
        let offset = if lh.peek(Token![@]) {
            let _: Token![@] = input.parse()?;

            if input.peek(Ident) {
                relative_base = Some(input.parse()?);
                let _: Token![+] = input.parse()?;
            }

            RegOffset::Fixed {
                offset: input.parse()?,
            }
        } else if lh.peek(Token![=>]) {
            let _: Token![=>] = input.parse()?;
            let mut alias: Path = input.parse()?;
            if input.peek(Token![+]) {
                let _: Token![+] = input.parse()?;
                relative_base = Some(alias);
                alias = input.parse()?;
            }

            if input.peek(token::Bracket) {
                let content;
                bracketed!(content in input);
                RegOffset::ElementAlias {
                    alias,
                    idx: content.parse()?,
                }
            } else {
                RegOffset::Alias { alias }
            }
        } else {
            Err(lh.error())?
        };

        let lh = input.lookahead1();
        let bitfield_args = if lh.peek(token::Brace) {
            input.parse()?
        } else {
            Err(lh.error())?
        };

        Ok(Self {
            attrs,
            vis,
            name,
            storage,
            array,
            relative_base,
            offset,
            bitfield_args,
        })
    }
}

pub(crate) struct RegDef {
    regs: Vec<Reg>,
}

impl Parse for RegDef {
    fn parse(input: syn::parse::ParseStream<'_>) -> Result<Self> {
        let mut regs = Vec::new();
        while !input.is_empty() {
            regs.push(input.parse()?);
        }
        Ok(RegDef { regs })
    }
}

pub(crate) fn register(def: RegDef) -> Result<TokenStream> {
    let mut outputs = TokenStream::new();

    for reg in def.regs {
        let Reg {
            attrs,
            vis,
            name,
            storage,
            array,
            relative_base,
            offset,
            bitfield_args,
        } = reg;

        // Use register name's span for generated code, so error messages (if any) can point to it
        // instead of the entire register allocation.
        let span = name.span().resolved_at(Span::mixed_site());

        let offset = match offset {
            RegOffset::Fixed { offset } => quote!(#offset),
            RegOffset::Alias { alias } => {
                quote_spanned!(alias.span().resolved_at(span) =>
                    <#alias as ::kernel::io::register::Register>::OFFSET
                )
            }
            RegOffset::ElementAlias { alias, idx } => {
                outputs.extend(quote_spanned!(idx.span().resolved_at(span) =>
                    ::kernel::build_assert::static_assert!(
                        (#idx) < <#alias as ::kernel::io::register::RegisterArray>::SIZE
                    );
                ));
                quote_spanned!(alias.span().resolved_at(span) =>
                    <#alias as ::kernel::io::register::Register>::OFFSET
                        + (#idx) * <#alias as ::kernel::io::register::RegisterArray>::STRIDE
                )
            }
        };

        outputs.extend(quote_spanned!(span =>
            ::kernel::bitfield!(
                // `#[allow(non_camel_case_types)]` is added since register names typically use
                // `SCREAMING_CASE`.
                #[allow(non_camel_case_types)]
                #(#attrs)* #vis struct #name(#storage) #bitfield_args
            );

            impl ::kernel::io::register::Register for #name {
                type Storage = #storage;

                const OFFSET: usize = #offset;
            }
        ));

        match array {
            None => match relative_base {
                None => outputs.extend(quote_spanned!(span =>
                    impl ::kernel::io::register::FixedRegister for #name {}

                    #(#attrs)* #vis const #name: ::kernel::io::register::FixedRegisterLoc<#name> =
                        ::kernel::io::register::FixedRegisterLoc::<#name>::new();
                )),
                Some(relative_base) => outputs.extend(quote_spanned!(span =>
                    impl ::kernel::io::register::WithBase for #name {
                        type BaseFamily = #relative_base;
                    }

                    impl ::kernel::io::register::RelativeRegister for #name {}
                )),
            },

            Some(def) => {
                let size = &def.size;
                let stride = if let Some(stride) = &def.stride {
                    outputs.extend(quote_spanned!(stride.span().resolved_at(span) =>
                        ::kernel::build_assert::static_assert!(
                            ::core::mem::size_of::<#storage>() <= #stride
                        );
                    ));
                    quote!(#stride)
                } else {
                    quote_spanned!(span => ::core::mem::size_of::<#storage>())
                };

                outputs.extend(quote_spanned!(span =>
                    impl ::kernel::io::register::RegisterArray for #name {
                        const SIZE: usize = #size;
                        const STRIDE: usize = #stride;
                    }
                ));

                match relative_base {
                    None => outputs.extend(quote_spanned!(span =>
                        impl ::kernel::io::register::Array for #name {}
                    )),
                    Some(relative_base) => outputs.extend(quote_spanned!(span =>
                        impl ::kernel::io::register::WithBase for #name {
                            type BaseFamily = #relative_base;
                        }

                        impl ::kernel::io::register::RelativeRegisterArray for #name {}
                    )),
                }
            }
        };
    }

    Ok(outputs)
}
