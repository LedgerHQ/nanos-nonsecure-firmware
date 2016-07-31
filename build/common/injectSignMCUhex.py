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
import argparse

def auto_int(x):
    return int(x, 0)

parser = argparse.ArgumentParser()
parser.add_argument("--hex", help="Hex file to be signed")
parser.add_argument("--signedHex", help="Output signed Hex file")
parser.add_argument("--signature", help="Signature to be injected (HEX)")
parser.add_argument("--sigAddr", help="Signature start address", type=auto_int)


args = parser.parse_args()

if args.hex == None:
	raise Exception("Missing hex filename to inject into")
if args.signedHex == None:
	raise Exception("Missing output injected hex filename")
if args.sigAddr == None:
	raise Exception("Missing signature start address")
if args.signature == None:
	raise Exception("Missing hex signature content")
# parse
parser = IntelHexParser(args.hex)
printer = IntelHexPrinter(parser)

#append sign and print out + ensure max padding
printer.addArea(args.sigAddr, args.signature.decode('hex')+"FFFFFFFF".decode('hex') )
printer.writeTo(args.signedHex)

