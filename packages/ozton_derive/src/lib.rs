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
        .filter_map(|field| {
            let skip = parse_field_attr(&field.attrs).unwrap_or(true);
            if skip {
                return None;
            }

            let field_ident = field
                .ident
                .as_ref()
                .expect("named fields ensured above")
                .clone();

            let field_type = &field.ty;

            Some(quote! {
                pub #field_ident: <#field_type as ozton_record::frame::FrameType>::Output,
            })
        })
        .collect::<Vec<_>>();

    // TODO: Add derives to serialize with rykv on the brain.

    let generated = quote! {
        pub struct #frame_ident {
            #(#fields)*
        }
    };

    Ok(generated.into())
}

fn parse_field_attr(attrs: &[Attribute]) -> Result<bool, syn::Error> {
    for attr in attrs {
        if !attr.path().is_ident("frame") {
            continue;
        }

        let args: syn::LitStr = attr.parse_args()?;

        println!("{args:?}");
    }

    Ok(false)
}
