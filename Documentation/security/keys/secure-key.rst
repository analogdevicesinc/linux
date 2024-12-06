==========
Secure Key
==========

Secure key is the new type added to kernel key ring service.
Secure key is a symmetric type key of minimum length 32 bytes
and with maximum possible length to be 128 bytes. It is produced
in kernel using the CAAM crypto engine. Userspace can only see
the blob for the corresponding key. All the blobs are displayed
or loaded in hex ascii.

Secure key can be created on platforms which supports CAAM
hardware block. Secure key can also be used as a master key to
create the encrypted keys along with the existing key types in
kernel.

Secure key uses CAAM hardware to generate the key and blobify its
content for userspace. Generated blobs are tied up with the hardware
secret key stored in CAAM, hence the same blob will not be able to
de-blobify with the different secret key on another machine.

Usage::

	keyctl add secure <name> "new <keylen>" <ring>
	keyctl load secure <name> "load <hex_blob>" <ring>
	keyctl print <key_id>

"keyctl add secure" option will create the random data of the
specified key len using CAAM and store it as a key in kernel.
Key contents will be displayed as blobs to the user in hex ascii.
User can input key len from 32 bytes to 128 bytes.

"keyctl load secure" option will load the blob contents. In kernel,
key will be deirved using input blob and CAAM, along with the secret
key stored in CAAM.

"keyctl print" will return the hex string of the blob corresponding to
key_id. Returned blob will be of key_len + 48 bytes. Extra 48 bytes are
the header bytes added by the CAAM.

Example of secure key usage::

1. Create the secure key with name kmk-master of length 32 bytes::

	$ keyctl add secure kmk-master "new 32" @u
	46001928

	$keyctl show
	Session Keyring
	1030783626 --alswrv      0 65534  keyring: _uid_ses.0
	 695927745 --alswrv      0 65534   \_ keyring: _uid.0
	  46001928 --als-rv      0     0       \_ secure: kmk-master

2. Print the blob contents for the kmk-master key::

	$ keyctl print 46001928
	d9743445b640f3d59c1670dddc0bc9c2
	34fc9aab7dd05c965e6120025012f029b
	07faa4776c4f6ed02899e35a135531e9a
	6e5c2b51132f9d5aef28f68738e658296
	3fe583177cfe50d2542b659a13039

	$ keyctl pipe 46001928 > secure_key.blob

3. Load the blob in the user key ring::

	$ keyctl load secure kmk-master "load 'cat secure_key.blob'" @u
