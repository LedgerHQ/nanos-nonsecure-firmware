"""
*******************************************************************************
*   Ledger Blue
*   (c) 2016 Ledger
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
********************************************************************************
"""

from ledgerblue.hexParser import IntelHexParser
from ledgerblue.hexParser import IntelHexPrinter
from secp256k1 import PrivateKey
import hashlib
import argparse

def auto_int(x):
    return int(x, 0)

parser = argparse.ArgumentParser()
parser.add_argument("--hex", help="Hex file to be signed")
parser.add_argument("--signedHex", help="Output signed Hex file")
parser.add_argument("--sigAddr", help="Signature start address", type=auto_int)
parser.add_argument("--key", help="The private key to sign with")


args = parser.parse_args()

if args.hex == None:
	raise Exception("Missing hex filename to sign")
if args.signedHex == None:
	raise Exception("Missing output signed hex filename")
if args.sigAddr == None:
	raise Exception("Missing signature start address")

# parse
parser = IntelHexParser(args.hex)
printer = IntelHexPrinter(parser)

# prepare data
dataToHash = ""
# consider areas are ordered by ascending address and non-overlaped
for a in parser.getAreas():
	dataToHash += a.data

m = hashlib.sha256()
m.update(dataToHash)
dataToSign = m.digest()

print "Hash: " + dataToSign.encode('hex')

# sign
signKey = PrivateKey(bytes(bytearray.fromhex(args.key)))
sign = signKey.ecdsa_sign(bytes(dataToSign), raw=True)
sign = signKey.ecdsa_serialize(sign)

#append sign and print out
printer.addArea(args.sigAddr, sign + "FFFFFFFF".decode('hex'))
printer.writeTo(args.signedHex)

