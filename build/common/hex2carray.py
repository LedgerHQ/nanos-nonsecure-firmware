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
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--hex", help="Hex file to be converted as a C array")

args = parser.parse_args()

if args.hex == None:
	raise Exception("Missing hex filename to sign")


parser = IntelHexParser(args.hex)

def hexU8(value):
	return hex(0x100|(value & 0xFF))[3:]

for a in parser.getAreas():

	if (len(a.data) > 0x10000):
		raise BaseException("data must be splitted in chunks of 64k")

	print "0x" + hexU8(a.start >> 24) + ", 0x" + hexU8(a.start >> 16) + ", 0x" + hexU8(a.start >> 8) + ", 0x" + hexU8(a.start) + ", "
	print "0x" + hexU8(len(a.data) >> 24) + ", 0x" + hexU8(len(a.data) >> 16) + ", 0x" + hexU8(len(a.data) >> 8) + ", 0x" + hexU8(len(a.data)) + ", "

	# low @ to high @
	offset = 0
	while offset < len(a.data):
		string = ""
		for i in range(8):
			if offset+i < len(a.data):
				string += " 0x" + hexU8(a.data[offset+i]) + "," 
		print string
		offset+=8

