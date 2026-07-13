// Verbatim excerpts of ore-edk-boot-opi 78479e21ba oreboot/src/mainboard/spacemit/k1x/bt0/src/main.rs
// (env_eq, env_u32, vendorid_to_name, impid_to_name), for the equivalence test.

const fn env_eq(v: Option<&str>, want: &str) -> bool {
    match v {
        None => false,
        Some(v) => {
            let (a, b) = (v.as_bytes(), want.as_bytes());
            if a.len() != b.len() {
                return false;
            }
            let mut i = 0;
            while i < a.len() {
                if a[i] != b[i] {
                    return false;
                }
                i += 1;
            }
            true
        }
    }
}

const fn env_u32(v: Option<&str>) -> u32 {
    match v {
        None => 0,
        Some(v) => {
            let b = v.as_bytes();
            let mut n = 0u32;
            let mut i = 0;
            while i < b.len() {
                n = n * 10 + (b[i] - b'0') as u32;
                i += 1;
            }
            n
        }
    }
}

fn vendorid_to_name<'a>(vendorid: usize) -> &'a str {
    match vendorid {
        0x0489 => "SiFive",
        0x0710 => "SpacemiT",
        _ => "unknown",
    }
}

fn impid_to_name<'a>(impid: usize) -> &'a str {
    match impid {
        0x0421_0427 => "21G1.02.00 / llama.02.00-general",
        0x1000_0000_4977_2200 => "X60",
        _ => "unknown",
    }
}
