#ifndef DECODER_H
#define DECODER_H

#include <bitset>
#include <vector>

#include "MarkerIDs.h"

namespace stag {

typedef std::bitset<48> Codeword;

class Decoder
{
	int wordSize = 48;
	int noOfCodewords;

	std::vector<Codeword> codewords;

public:
	Decoder(){}
	Decoder(int hd);
	bool decode(const Codeword& c, int errCorr, int& id, int& shift);
};

} // namespace stag

#endif