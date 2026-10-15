//! Public development keys for the SpacemiT K1 BootROM image.
//!
//! These keys are NOT security keys. Their private halves are published in
//! this repository on purpose, so anyone can rebuild a bit-identical bt0.
//! They provide reproducible image-format test material. Acceptance of these
//! replacement keys by any board, and fused/unfused ROM behavior, are separate
//! unverified physical gates. Never provision these public keys as a trust root.
//!
//! They belong to this project and are not SpacemiT's sample keys. Each key
//! is derived from a fixed seed (`cargo xtask k1x-dev-keys` rewrites the PEM
//! files under `keys/`, `--check` compares them) and committed as PKCS#1 PEM,
//! so a later change in the RSA crate's key generation cannot change the
//! keys a build uses.

use std::{fs, path::PathBuf, process};

use log::{error, info};
use rand::SeedableRng;
use rand_chacha::ChaCha20Rng;
use rsa::pkcs1::{DecodeRsaPrivateKey, EncodeRsaPrivateKey, LineEnding};
use rsa::sha2::{Digest, Sha256};
use rsa::RsaPrivateKey;

/// RSA modulus size the BootROM expects.
pub const KEY_BITS: usize = 2048;

/// Seed prefix; the role name follows it.
const SEED_TAG: &str = "ore-edk-boot-opi K1 BootROM public development key v1: ";

/// The two keys of a K1 BootROM image.
#[derive(Clone, Copy, Debug)]
pub enum Role {
    /// Root key (ROTPK): its modulus opens the image and it signs the
    /// manifest (header0, key config, SPL key).
    Root,
    /// SPL (OEM) key: its modulus is in the manifest and it signs header1
    /// and the payload.
    Spl,
}

pub const ROLES: [Role; 2] = [Role::Root, Role::Spl];

impl Role {
    fn name(self) -> &'static str {
        match self {
            Role::Root => "root",
            Role::Spl => "spl",
        }
    }

    fn file_name(self) -> String {
        format!("k1x-dev-{}.pem", self.name())
    }

    fn pem(self) -> &'static str {
        match self {
            Role::Root => include_str!("keys/k1x-dev-root.pem"),
            Role::Spl => include_str!("keys/k1x-dev-spl.pem"),
        }
    }

    /// The committed key.
    pub fn key(self) -> RsaPrivateKey {
        RsaPrivateKey::from_pkcs1_pem(self.pem()).expect("committed K1 development key")
    }

    /// Derive the key from its seed: SHA-256 of the seed tag and role name
    /// seeds ChaCha20, which drives the RSA crate's key generation
    /// (public exponent 65537).
    pub fn derive(self) -> RsaPrivateKey {
        let seed: [u8; 32] = Sha256::digest(format!("{SEED_TAG}{}", self.name())).into();
        let mut rng = ChaCha20Rng::from_seed(seed);
        RsaPrivateKey::new(&mut rng, KEY_BITS).expect("RSA key generation")
    }

    fn derive_pem(self) -> String {
        self.derive()
            .to_pkcs1_pem(LineEnding::LF)
            .expect("PKCS#1 PEM encoding")
            .to_string()
    }
}

fn keys_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src/spacemit/keys")
}

/// `cargo xtask k1x-dev-keys [--check]`: derive the keys, and write them
/// into the source tree or compare them with the committed files.
pub fn run(check: bool, directory: Option<PathBuf>) {
    let dir = directory.unwrap_or_else(keys_dir);
    if !check {
        fs::create_dir_all(&dir).expect("create public development-key directory");
    }
    let mut mismatch = false;
    for role in ROLES {
        let path = dir.join(role.file_name());
        let pem = role.derive_pem();
        if check {
            let committed = fs::read_to_string(&path).unwrap_or_default();
            if committed == pem {
                info!("{}: matches the derived key", path.display());
            } else {
                error!("{}: differs from the derived key", path.display());
                mismatch = true;
            }
        } else {
            fs::write(&path, &pem).expect("write key file");
            info!("{}: written", path.display());
        }
    }
    if mismatch {
        process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rsa::traits::PublicKeyParts;
    use rsa::BigUint;

    #[test]
    fn committed_keys_are_derived() {
        for role in ROLES {
            assert_eq!(role.pem(), role.derive_pem(), "{} key", role.name());
        }
    }

    #[test]
    fn committed_keys_shape() {
        let root = Role::Root.key();
        let spl = Role::Spl.key();
        for k in [&root, &spl] {
            assert_eq!(k.n().bits(), KEY_BITS);
            assert_eq!(*k.e(), BigUint::from(65537u32));
            k.validate().unwrap();
        }
        assert_ne!(root.n(), spl.n());
    }
}
