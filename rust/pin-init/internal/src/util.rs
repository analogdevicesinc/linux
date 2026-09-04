// SPDX-License-Identifier: Apache-2.0 OR MIT

use proc_macro2::TokenStream;
use syn::Attribute;

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
