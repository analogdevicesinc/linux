// SPDX-License-Identifier: Apache-2.0 OR MIT

use proc_macro2::{Ident, TokenStream};
use quote::format_ident;
use syn::{Attribute, Index, Member};

pub(crate) trait AttrListExt {
    fn extract_cfg_attrs(&mut self) -> Vec<TokenStream>;
}

impl AttrListExt for Vec<Attribute> {
    fn extract_cfg_attrs(&mut self) -> Vec<TokenStream> {
        let cfg: Vec<_> = self
            .iter()
            .filter(|a| a.path().is_ident("cfg"))
            .map(|a| {
                a.parse_args::<TokenStream>()
                    .expect("parse as token stream cannot fail")
            })
            .collect();

        if !cfg.is_empty() {
            self.retain(|a| !a.path().is_ident("cfg"));
        }

        cfg
    }
}

pub(crate) trait MemberExt {
    /// Returns an identifier for the member.
    ///
    /// Tuple fields have no name of their own, so they are named `_0`, `_1`, ... instead.
    fn as_ident(&self) -> Ident;

    /// Obtain a display name for the member in diagnostics.
    fn display_name(&self) -> String;
}

impl MemberExt for Member {
    fn as_ident(&self) -> Ident {
        match self {
            Member::Named(ident) => ident.clone(),
            Member::Unnamed(Index { index, .. }) => format_ident!("_{index}"),
        }
    }

    fn display_name(&self) -> String {
        match self {
            Member::Named(ident) => format!("`{ident}`"),
            Member::Unnamed(Index { index, .. }) => format!("index `{index}`"),
        }
    }
}
