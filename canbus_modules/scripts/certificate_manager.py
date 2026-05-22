#!/usr/bin/env python3
import os
import tempfile
import ipaddress
from typing import Optional, Tuple

from cryptography import x509
from cryptography.x509.oid import NameOID
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from datetime import datetime, timedelta, timezone


def generate_server_cert() -> Tuple[Optional[str], Optional[str]]:
    try:
        private_key = rsa.generate_private_key(
            public_exponent=65537,
            key_size=2048,
        )

        subject = issuer = x509.Name([
            x509.NameAttribute(NameOID.COUNTRY_NAME, "PL"),
            x509.NameAttribute(NameOID.STATE_OR_PROVINCE_NAME, "Mazowieckie"),
            x509.NameAttribute(NameOID.LOCALITY_NAME, "Warszawa"),
            x509.NameAttribute(NameOID.ORGANIZATION_NAME, "Web Robot Control"),
            x509.NameAttribute(NameOID.COMMON_NAME, "localhost"),
        ])

        cert = (x509.CertificateBuilder().subject_name(subject).issuer_name(
            issuer).public_key(private_key.public_key()).serial_number(
                x509.random_serial_number()).not_valid_before(
                    datetime.now(timezone.utc)).not_valid_after(
                        datetime.now(timezone.utc) +
                        timedelta(days=365)).add_extension(
                            x509.SubjectAlternativeName([
                                x509.IPAddress(
                                    ipaddress.IPv4Address("127.0.0.1")),
                                x509.IPAddress(ipaddress.IPv6Address("::1")),
                                x509.DNSName("localhost"),
                            ]),
                            critical=False,
                        ).sign(private_key, hashes.SHA256()))

        cert_file = tempfile.NamedTemporaryFile(delete=False, suffix=".crt")
        key_file = tempfile.NamedTemporaryFile(delete=False, suffix=".key")

        cert_file.write(cert.public_bytes(serialization.Encoding.PEM))
        key_file.write(
            private_key.private_bytes(
                encoding=serialization.Encoding.PEM,
                format=serialization.PrivateFormat.PKCS8,
                encryption_algorithm=serialization.NoEncryption(),
            ))

        cert_file.close()
        key_file.close()

        return cert_file.name, key_file.name

    except Exception as e:
        return None, None


def cleanup_certificate_files(*file_paths: str) -> None:
    for file_path in file_paths:
        if file_path and os.path.exists(file_path):
            try:
                os.unlink(file_path)
            except Exception as e:
                pass
