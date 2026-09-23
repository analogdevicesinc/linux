// SPDX-License-Identifier: Apache-2.0 OR MIT

use std::fmt::Display;

use proc_macro2::TokenStream;
use quote::{quote, quote_spanned};
use syn::{spanned::Spanned, Error};

pub(crate) struct DiagCtxt(TokenStream);
pub(crate) struct ErrorGuaranteed(());

impl DiagCtxt {
    pub(crate) fn error(&mut self, span: impl Spanned, msg: impl Display) -> ErrorGuaranteed {
        let error = Error::new(span.span(), msg);
        self.0.extend(error.into_compile_error());
        ErrorGuaranteed(())
    }

    pub(crate) fn warn(&mut self, span: impl Spanned, msg: impl Display) {
        // Have the message start on a new line for visual clarity.
        let msg = format!("\n{}", msg);
        self.0.extend(quote_spanned!(span.span() =>
            // Approximate using deprecated warning while `proc_macro_diagnostic` is unstable.
            const _: () = {
                #[deprecated = #msg]
                const fn warn() {}
                warn();
            };
        ));
    }

    fn with(
        f: impl FnOnce(&mut DiagCtxt) -> Result<TokenStream, ErrorGuaranteed>,
        merge_diag: impl FnOnce(TokenStream, TokenStream) -> TokenStream,
        convert_diag: impl FnOnce(TokenStream) -> TokenStream,
    ) -> TokenStream {
        let mut dcx = Self(TokenStream::new());
        match f(&mut dcx) {
            Ok(stream) => {
                if dcx.0.is_empty() {
                    stream
                } else {
                    merge_diag(stream, dcx.0)
                }
            }
            Err(ErrorGuaranteed(())) => convert_diag(dcx.0),
        }
    }

    pub(crate) fn for_item(
        f: impl FnOnce(&mut DiagCtxt) -> Result<TokenStream, ErrorGuaranteed>,
    ) -> TokenStream {
        Self::with(
            f,
            |mut out, diag| {
                out.extend(diag);
                out
            },
            std::convert::identity,
        )
    }

    pub(crate) fn for_expr(
        f: impl FnOnce(&mut DiagCtxt) -> Result<TokenStream, ErrorGuaranteed>,
    ) -> TokenStream {
        Self::with(
            f,
            |out, diag| {
                // Diagnostics that we generate are always items.
                // So for expressions create a block to place diagnostics in item position.
                quote!({
                    #diag
                    #out
                })
            },
            |diag| {
                quote!({
                    #diag
                })
            },
        )
    }
}
