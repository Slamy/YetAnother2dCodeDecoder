/*
 * test_qrcode.cpp
 *
 *  Created on: 04.11.2021
 *      Author: andre
 */

#include "QrDecoder.h"
#include "gtest/gtest.h"

struct GroundTruth
{
	const char* filename;
	const char* reference_payload;
	bool syndromesExpectedZero;
};

struct GroundTruth data[] = {
	{"Japan-qr-code-billboard.jpg",
	 "http://sagasou.mobi "
	 "\r\n\r\nMEBKM:TITLE:\x92T\x82\xBB\x82\xA4\x83\x82\x83r\x82"
	 "\xC5\x90\xEA\x96\xE5\x8Aw\x8DZ\x92T\x82\xB5\x81I;URL:"
	 "http\\://sagasou.mobi;;",
	 true},
	{"ArminHanisch.png", "https://ArminHanisch.de", true},													//
	{"inmycar2.png", "mediamarkt.de/amua", false},															//
	{"inmycar.jpg", "mediamarkt.de/amua", false},															//
	{"chrome_dino.png", "https://www.googlewatchblog.de/2020/07/google-chrome-so-hintergrundbild/", false}, //
	{"wikipedia_perspective.png", "https://de.wikipedia.org", true},										//
	{"wikipedia.png", "https://de.wikipedia.org", true},													//
	{"numeric.png", "tel:555-555-5555", true},																//
	{"numeric2.png", "tel:0043-7252-72720", true},															//
	{"numeric3.png", "tel:00431789", true},																	//
	{"numeric4.png", "tel:0043", false}, // TODO false can't be. there shouldn't be an error in the code
	{"Qr-projekt-taunusanlage-beethoven-denkmal-2011-ffm-029.jpg", "www.kunst-in-ffm.de/mobile/obj114.html", false}, //
	{"sepa_multi_mode.png",
	 "BCD\n002\n1\nSCT\nRLNWATWW\n\xC3\x84rzte ohne Grenzen\nAT973200000000518548\n\n\n\nSpende\nSpende f\xC3\xBCr MSF "
	 "Nothilfe",
	 true},																	//
	{"schild_qrcode.jpg", "http://artloops.ch/art-loop-04/e/", true},		//
	{"wifi_key.png", "WIFI:T:WPA;S:Netzwerkname (SSID);P:HowdyHo;;", true}, //
};

TEST(QrTest, Validate)
{
	QrDecoder qr;
	std::vector<char> result;

	for (auto& g : data)
	{
		std::stringstream path;
		path << "/home/andre/GIT/QrCode/qrcode_ref/" << g.filename;

		std::cout << "Reading " << g.filename << std::endl;
		result = qr.decodeFromFile(path.str().c_str());

		result.push_back('\0');
		ASSERT_STREQ(result.data(), g.reference_payload);

		ASSERT_EQ(qr.code_needed_correction_, !g.syndromesExpectedZero);

		std::cout << std::endl;
	}
}
