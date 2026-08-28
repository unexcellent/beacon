//! Chip drivers, generic over `embedded-hal` buses.

mod mi48;
mod sc850sl;

pub use mi48::Mi48;
pub use sc850sl::Sc850sl;
