extern crate proc_macro;

use proc_macro::TokenStream;
use proc_macro_crate::{FoundCrate, crate_name};
use quote::{format_ident, quote};
use syn::{Attribute, DeriveInput, Path, parse_macro_input};

#[proc_macro_derive(RecordedRobot, attributes(record))]
pub fn recorded_robot_derive(input: TokenStream) -> TokenStream {
    let ast = parse_macro_input!(input as DeriveInput);

    match impl_recorded_robot(&ast) {
        Ok(tokens) => tokens,
        Err(error) => error.to_compile_error().into(),
    }
}

fn impl_recorded_robot(ast: &DeriveInput) -> Result<TokenStream, syn::Error> {
    let name = &ast.ident;
    let frame_ident = format_ident!("{}Frame", name);
    let record_path = resolve_record_path()?;

    let fields = match &ast.data {
        syn::Data::Struct(data_struct) => match &data_struct.fields {
            syn::Fields::Named(fields_named) => &fields_named.named,
            _ => {
                return Err(syn::Error::new_spanned(
                    &ast.ident,
                    "RecordedRobot only supports named fields",
                ));
            }
        },
        _ => {
            return Err(syn::Error::new_spanned(
                &ast.ident,
                "RecordedRobot can only be derived for structs",
            ));
        }
    };

    let included_fields = fields
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

            Ok(Some((field_ident, field.ty.clone())))
        })
        .collect::<Result<Vec<_>, syn::Error>>()?
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();

    let frame_fields = included_fields.iter().map(|(field_ident, field_type)| {
        quote! {
            pub #field_ident: <#field_type as #record_path::FrameType>::Output,
        }
    });

    let interpolate_fields = included_fields.iter().map(|(field_ident, _)| {
        quote! {
            #field_ident: #record_path::Interpolate::interpolate(
                &from.#field_ident,
                &to.#field_ident,
                amount,
            ),
        }
    });

    let finalize_fields = included_fields.iter().map(|(field_ident, field_type)| {
        quote! {
            #field_ident: <#field_type as #record_path::RecordField>::finalize_frame_value(
                &self.#field_ident,
                &frame.#field_ident,
            ).await,
        }
    });

    let apply_fields = included_fields.iter().map(|(field_ident, field_type)| {
        quote! {
            <#field_type as #record_path::RecordField>::apply_frame_value(
                &mut self.#field_ident,
                &frame.#field_ident,
                mode,
            ).await?;
        }
    });

    let record_field_bounds = included_fields.iter().map(|(_, field_type)| {
        quote! {
            #field_type: #record_path::RecordField,
        }
    });

    let where_clause = if included_fields.is_empty() {
        quote! {}
    } else {
        quote! {
            where
                #(#record_field_bounds)*
        }
    };

    let generated = quote! {
        #[derive(
            #record_path::rkyv::Archive,
            #record_path::rkyv::Serialize,
            #record_path::rkyv::Deserialize,
            Default,
            Clone,
            Debug
        )]
        #[rkyv(crate = #record_path::rkyv)]
        pub struct #frame_ident {
            #(#frame_fields)*
        }

        impl #record_path::Interpolate for #frame_ident {
            fn interpolate(from: &Self, to: &Self, amount: f64) -> Self {
                Self {
                    #(#interpolate_fields)*
                }
            }
        }

        #[#record_path::async_trait(?Send)]
        impl #record_path::frame::FrameRobot for #name
        #where_clause
        {
            type Frame = #frame_ident;

            async fn finalize_frame(&self, frame: &Self::Frame) -> Self::Frame {
                #frame_ident {
                    #(#finalize_fields)*
                }
            }

            async fn apply_frame(
                &mut self,
                frame: &Self::Frame,
                mode: #record_path::frame::RecordMode,
            ) -> Result<(), #record_path::PortError> {
                #(#apply_fields)*
                Ok(())
            }
        }
    };

    Ok(generated.into())
}

fn resolve_record_path() -> Result<Path, syn::Error> {
    if let Ok(found) = crate_name("ozton") {
        return Ok(match found {
            FoundCrate::Itself => syn::parse_quote!(crate::record),
            FoundCrate::Name(name) => {
                let ident = format_ident!("{}", name);
                syn::parse_quote!(::#ident::record)
            }
        });
    }

    if let Ok(found) = crate_name("ozton-record") {
        return Ok(match found {
            FoundCrate::Itself => syn::parse_quote!(crate),
            FoundCrate::Name(name) => {
                let ident = format_ident!("{}", name);
                syn::parse_quote!(::#ident)
            }
        });
    }

    Err(syn::Error::new(
        proc_macro2::Span::call_site(),
        "RecordedRobot derive requires either `ozton` or `ozton-record` to be a direct dependency",
    ))
}

fn parse_field_attr(attrs: &[Attribute]) -> Result<bool, syn::Error> {
    let mut skip = false;

    for attr in attrs {
        if !attr.path().is_ident("record") {
            continue;
        }

        attr.parse_nested_meta(|meta| {
            if meta.path.is_ident("skip") {
                skip = true;
                return Ok(());
            }

            Err(meta.error("unsupported record attribute; expected `skip`"))
        })?;
    }

    Ok(skip)
}

#[cfg(test)]
mod tests {
    use syn::parse_quote;

    use super::parse_field_attr;

    #[test]
    fn record_skip_sets_skip_true() {
        let attrs = vec![parse_quote!(#[record(skip)])];
        assert!(parse_field_attr(&attrs).unwrap());
    }

    #[test]
    fn no_record_attr_does_not_skip() {
        let attrs = vec![parse_quote!(#[allow(dead_code)])];
        assert!(!parse_field_attr(&attrs).unwrap());
    }

    #[test]
    fn unknown_record_attr_errors() {
        let attrs = vec![parse_quote!(#[record(foo)])];
        assert!(parse_field_attr(&attrs).is_err());
    }
}
