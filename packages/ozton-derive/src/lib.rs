extern crate proc_macro;

use proc_macro::TokenStream;
use quote::{format_ident, quote};
use syn::{Attribute, DeriveInput, parse_macro_input};

#[proc_macro_derive(RobotFrame, attributes(frame))]
pub fn robot_frame_derive(input: TokenStream) -> TokenStream {
    let ast = parse_macro_input!(input as DeriveInput);

    match impl_robot_frame(&ast) {
        Ok(x) => x,
        Err(e) => e.to_compile_error().into(),
    }
}

fn impl_robot_frame(ast: &syn::DeriveInput) -> Result<TokenStream, syn::Error> {
    let name = &ast.ident;
    let frame_ident = format_ident!("{}Frame", name);

    let fields = match &ast.data {
        syn::Data::Struct(data_struct) => match &data_struct.fields {
            syn::Fields::Named(fields_named) => &fields_named.named,

            _ => {
                return Err(syn::Error::new_spanned(
                    &ast.ident,
                    "RobotFrame only supports named fields",
                ));
            }
        },
        _ => {
            return Err(syn::Error::new_spanned(
                &ast.ident,
                "RobotFrame can only be derived for structs",
            ));
        }
    };

    let fields = fields
        .iter()
        .map(|field| {
            let skip = parse_field_attr(&field.attrs)?;
            if skip {
                return Ok(None);
            }

            let field_ident = field
                .ident
                .as_ref()
                .expect("named fields ensured above")
                .clone();

            let field_type = &field.ty;

            Ok(Some(quote! {
                pub #field_ident: <#field_type as ::ozton_record::frame_types::FrameType>::Output,
            }))
        })
        .collect::<Result<Vec<_>, syn::Error>>()?
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();

    let generated = quote! {
        #[derive(::ozton_record::rkyv::Archive, ::ozton_record::rkyv::Serialize, ::ozton_record::rkyv::Deserialize, Default, Clone, Debug)]
        pub struct #frame_ident {
            #(#fields)*
        }
    };

    Ok(generated.into())
}

fn parse_field_attr(attrs: &[Attribute]) -> Result<bool, syn::Error> {
    let mut skip = false;

    for attr in attrs {
        if !attr.path().is_ident("frame") {
            continue;
        }

        attr.parse_nested_meta(|meta| {
            if meta.path.is_ident("skip") {
                skip = true;
                return Ok(());
            }

            Err(meta.error("unsupported frame attribute; expected `skip`"))
        })?;
    }

    Ok(skip)
}

#[cfg(test)]
mod tests {
    use syn::parse_quote;

    use super::parse_field_attr;

    #[test]
    fn frame_skip_sets_skip_true() {
        let attrs = vec![parse_quote!(#[frame(skip)])];
        assert!(parse_field_attr(&attrs).unwrap());
    }

    #[test]
    fn no_frame_attr_does_not_skip() {
        let attrs = vec![parse_quote!(#[allow(dead_code)])];
        assert!(!parse_field_attr(&attrs).unwrap());
    }

    #[test]
    fn unknown_frame_attr_errors() {
        let attrs = vec![parse_quote!(#[frame(foo)])];
        assert!(parse_field_attr(&attrs).is_err());
    }
}
