//! # ozton
//!

#![cfg_attr(docsrs, feature(doc_cfg))]

#[doc(inline)]
#[cfg(feature = "derive")]
pub use ozton_derive as derive;
#[doc(inline)]
#[cfg(feature = "record")]
pub use ozton_record as record;

pub mod prelude {}
