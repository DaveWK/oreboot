//! SpacemiT K1 BootROM image ("AIHD") packer.
//!
//! Image layout:
//!
//! | Offset | Size  | Content                                              |
//! |--------|-------|------------------------------------------------------|
//! | 0x000  | 0x100 | root key (ROTPK) RSA-2048 modulus, big-endian        |
//! | 0x100  | 0x020 | header0                                              |
//! | 0x120  | 0x1e0 | key config, zero-padded                              |
//! | 0x300  | 0x100 | SPL key RSA-2048 modulus, big-endian                 |
//! | 0x400  | 0x700 | reserved, zero                                       |
//! | 0xb00  | 0x100 | signature0: root key over 0x100..0xb00               |
//! | 0xc00  | 0x3e0 | reserved, zero                                       |
//! | 0xfe0  | 0x020 | header1                                              |
//! | 0x1000 |       | payload, always adding 1..32 zero bytes of alignment |
//! | end    | 0x100 | signature1: SPL key over header1 and padded payload  |
//!
//! Signatures are RSASSA-PKCS1-v1_5 with SHA-256; the public exponent, 65537,
//! is not stored in the image. The keys are this project's public
//! development keys (`k1x_keys`). BootROM acceptance of these replacement
//! keys and fused/unfused behavior require separate physical evidence; these
//! signatures establish no production trust root or authenticated later chain.

use log::trace;
use rsa::pkcs1v15::SigningKey;
use rsa::sha2::Sha256;
use rsa::signature::{SignatureEncoding, Signer};
use rsa::traits::PublicKeyParts;
use rsa::RsaPrivateKey;
use zerocopy::IntoBytes;

use super::k1x_keys::Role;

const DEBUG: bool = false;

const AIHD: &[u8; 4] = b"AIHD";

#[derive(Clone, Copy, Debug)]
#[repr(C)]
struct Header {
    magic: [u8; 4],
    version: u8,
    secure: u8,
    reserved: [u8; 2],
    img_size: u64,
    load_addr: u64,
    pad: [u8; 8],
}

impl Header {
    fn encode(self) -> [u8; 32] {
        let mut bytes = [0u8; 32];
        bytes[..4].copy_from_slice(&self.magic);
        bytes[4] = self.version;
        bytes[5] = self.secure;
        bytes[6..8].copy_from_slice(&self.reserved);
        bytes[8..16].copy_from_slice(&self.img_size.to_le_bytes());
        bytes[16..24].copy_from_slice(&self.load_addr.to_le_bytes());
        bytes[24..].copy_from_slice(&self.pad);
        bytes
    }
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
struct Config {
    key_default: u32,
    table_num: u32,
    key_name: [u8; 16],
    key_id: u32,
}

const KEY_NAME_MAX_LEN: usize = 16;

impl Config {
    fn encode(self) -> [u8; 28] {
        let mut bytes = [0u8; 28];
        bytes[..4].copy_from_slice(&self.key_default.to_le_bytes());
        bytes[4..8].copy_from_slice(&self.table_num.to_le_bytes());
        bytes[8..24].copy_from_slice(&self.key_name);
        bytes[24..].copy_from_slice(&self.key_id.to_le_bytes());
        bytes
    }
    fn new(key_name: &str) -> Self {
        let l = key_name.len();
        assert!(l <= KEY_NAME_MAX_LEN);
        let mut kn = [0u8; KEY_NAME_MAX_LEN];
        kn[0..l].copy_from_slice(key_name.as_bytes());

        Self {
            key_default: 0,
            table_num: 2,
            key_name: kn,
            key_id: 0,
        }
    }
}

const HC_ADDR: u64 = 0x100;
const HC_SIZE: u64 = 0x200;

fn dump(s: &[u8]) {
    let chunk_size = 16;
    for (i, c) in s.chunks(chunk_size).enumerate() {
        println!("{:08x}: {c:02x?}", i * chunk_size);
    }
}

fn align(s: &[u8], alignment: usize) -> Vec<u8> {
    let len = s.len();
    let aligned_size = (len / alignment + 1) * alignment;
    let mut v = s.to_vec();
    v.resize(aligned_size, 0);
    v
}

// Create manifest, which includes the modulus of the payload signature key, and sign it.
fn create_signed_manifest(key: RsaPrivateKey, payload_key_mod: &[u8]) -> Vec<u8> {
    let h = Header {
        magic: *AIHD,
        version: 1,
        secure: 0,
        reserved: [0, 0],
        img_size: HC_SIZE,
        load_addr: HC_ADDR,
        pad: [0xa5u8; 8],
    };
    let c = Config::new("spl");
    let mut header_and_config = [&h.encode()[..], &c.encode()[..]].concat();
    header_and_config.resize(HC_SIZE as usize, 0);

    // What to sign: header + config + payload key modulus + reserved space
    let manifest = [header_and_config.as_bytes(), payload_key_mod, &[0u8; 0x700]].concat();

    // PKCS#1 v1.5 signatures are deterministic.
    let signature = SigningKey::<Sha256>::new(key).sign(&manifest);
    trace!("{signature:02x?}");

    [manifest, signature.to_vec()].concat()
}

const PAYLOAD_ALIGNMENT: usize = 32;

fn image(
    payload: &[u8],
    payload_key: RsaPrivateKey,  // key to sign the payload with
    manifest_key: RsaPrivateKey, // key to sign the manifest with
) -> Vec<u8> {
    let manifest_mod = manifest_key.n().to_bytes_be();
    let payload_mod = payload_key.n().to_bytes_be();

    let signed_manifest = create_signed_manifest(manifest_key, &payload_mod);

    // Align payload and create header, then sign them
    let payload_aligned = align(payload, PAYLOAD_ALIGNMENT);
    let load_addr = HC_ADDR + HC_SIZE;
    let h = Header {
        magic: *AIHD,
        version: 1,
        secure: 0,
        reserved: [0, 0],
        img_size: payload_aligned.len() as u64,
        load_addr,
        pad: [0xa5u8; 8],
    };
    let header_and_aligned_payload = [&h.encode()[..], &payload_aligned[..]].concat();

    let signature = SigningKey::<Sha256>::new(payload_key).sign(&header_and_aligned_payload);
    trace!("{signature:02x?}");

    // Concat it all
    [
        &manifest_mod,
        signed_manifest.as_bytes(),
        &[0u8; (0x400 - 32)],
        &header_and_aligned_payload,
        &signature.to_bytes(),
    ]
    .concat()
}

pub fn build(payload: &[u8]) -> Vec<u8> {
    let img = image(payload, Role::Spl.key(), Role::Root.key());

    if DEBUG {
        dump(&img);
    }

    img
}

#[cfg(test)]
mod tests {
    use super::*;
    use rsa::sha2::Digest;
    use rsa::BigUint;

    // Independent encoded-message check: no rsa::verify or signing implementation.
    fn raw_verify(modulus: &[u8], message: &[u8], signature: &[u8]) -> bool {
        if modulus.len() != 256 || signature.len() != 256 {
            return false;
        }
        let n = BigUint::from_bytes_be(modulus);
        let s = BigUint::from_bytes_be(signature);
        if n.bits() != 2048 || s >= n {
            return false;
        }
        let encoded = s.modpow(&BigUint::from(65537u32), &n).to_bytes_be();
        let mut got = vec![0; 256 - encoded.len()];
        got.extend(encoded);
        let digest_info = [
            &b"\x30\x31\x30\x0d\x06\x09\x60\x86\x48\x01\x65\x03\x04\x02\x01\x05\x00\x04\x20"[..],
            &Sha256::digest(message)[..],
        ]
        .concat();
        let mut expected = vec![0, 1];
        expected.resize(256 - digest_info.len() - 1, 0xff);
        expected.push(0);
        expected.extend(digest_info);
        got == expected
    }

    #[test]
    fn exact_layout_raw_rsa_and_repeated_packing() {
        for len in [0, 1, 31, 32, 33, 255, 256, 4096] {
            let payload: Vec<u8> = (0..len).map(|i| (i * 13 + 7) as u8).collect();
            let bytes = build(&payload);
            assert_eq!(bytes, build(&payload));
            let padded = (len / 32 + 1) * 32; // Existing format always adds1..32.
            assert_eq!(bytes.len(), 0x1000 + padded + 256);
            assert_eq!(&bytes[0x100..0x108], b"AIHD\x01\x00\x00\x00");
            assert_eq!(
                u64::from_le_bytes(bytes[0x108..0x110].try_into().unwrap()),
                0x200
            );
            assert_eq!(
                u64::from_le_bytes(bytes[0x110..0x118].try_into().unwrap()),
                0x100
            );
            assert_eq!(&bytes[0x118..0x120], &[0xa5; 8]);
            assert_eq!(&bytes[0x120..0x128], &[0, 0, 0, 0, 2, 0, 0, 0]);
            assert_eq!(&bytes[0x128..0x12b], b"spl");
            assert!(bytes[0x12b..0x300].iter().all(|b| *b == 0));
            assert_eq!(&bytes[..256], Role::Root.key().n().to_bytes_be());
            assert_eq!(&bytes[0x300..0x400], Role::Spl.key().n().to_bytes_be());
            assert!(bytes[0x400..0xb00].iter().all(|b| *b == 0));
            assert!(bytes[0xc00..0xfe0].iter().all(|b| *b == 0));
            assert_eq!(&bytes[0xfe0..0xfe8], b"AIHD\x01\x00\x00\x00");
            assert_eq!(
                u64::from_le_bytes(bytes[0xfe8..0xff0].try_into().unwrap()),
                padded as u64
            );
            assert_eq!(
                u64::from_le_bytes(bytes[0xff0..0xff8].try_into().unwrap()),
                0x300
            );
            assert_eq!(&bytes[0xff8..0x1000], &[0xa5; 8]);
            assert_eq!(&bytes[0x1000..0x1000 + len], payload);
            assert!(bytes[0x1000 + len..0x1000 + padded].iter().all(|b| *b == 0));
            assert!(raw_verify(
                &bytes[..256],
                &bytes[0x100..0xb00],
                &bytes[0xb00..0xc00]
            ));
            assert!(raw_verify(
                &bytes[0x300..0x400],
                &bytes[0xfe0..bytes.len() - 256],
                &bytes[bytes.len() - 256..]
            ));
        }
    }

    #[test]
    fn raw_rsa_rejects_independent_message_key_and_signature_changes() {
        let bytes = build(b"independent K1 development-signature failure controls");
        for (modulus, message, signature) in [
            (&bytes[..256], &bytes[0x100..0xb00], &bytes[0xb00..0xc00]),
            (
                &bytes[0x300..0x400],
                &bytes[0xfe0..bytes.len() - 256],
                &bytes[bytes.len() - 256..],
            ),
        ] {
            for index in [0, message.len() / 2, message.len() - 1] {
                let mut bad = message.to_vec();
                bad[index] ^= 1;
                assert!(!raw_verify(modulus, &bad, signature));
            }
            let mut bad = signature.to_vec();
            bad[200] ^= 1;
            assert!(!raw_verify(modulus, message, &bad));
            assert!(!raw_verify(modulus, message, &signature[..255]));
            assert!(!raw_verify(modulus, message, modulus)); // signature>=n
            let mut bad_key = modulus.to_vec();
            bad_key[255] ^= 2;
            assert!(!raw_verify(&bad_key, message, signature));
        }
        // A root signature cannot be verified with the distinct SPL key.
        assert!(!raw_verify(
            &bytes[0x300..0x400],
            &bytes[0x100..0xb00],
            &bytes[0xb00..0xc00]
        ));
    }

    #[test]
    fn fields_encode_little_endian_without_native_struct_layout() {
        let h = Header {
            magic: *b"AIHD",
            version: 0x12,
            secure: 0x34,
            reserved: [0x56, 0x78],
            img_size: 0x0102030405060708,
            load_addr: 0x1112131415161718,
            pad: [0xa5; 8],
        }
        .encode();
        assert_eq!(
            &h[4..24],
            &[
                0x12, 0x34, 0x56, 0x78, 8, 7, 6, 5, 4, 3, 2, 1, 0x18, 0x17, 0x16, 0x15, 0x14, 0x13,
                0x12, 0x11
            ]
        );
        let c = Config {
            key_default: 0x01020304,
            table_num: 0x11121314,
            key_name: [0x55; 16],
            key_id: 0x21222324,
        }
        .encode();
        assert_eq!(&c[..8], &[4, 3, 2, 1, 0x14, 0x13, 0x12, 0x11]);
        assert_eq!(&c[24..], &[0x24, 0x23, 0x22, 0x21]);
    }
}
