/*
 * qrcodeexception.h
 *
 *  Created on: 12.11.2021
 *      Author: andre
 */

#ifndef QRCODEEXCEPTION_H_
#define QRCODEEXCEPTION_H_

#include <exception>

class QrCodeException : public std::exception
{
  public:
	enum class Cause
	{
		kFileNotReadable,
		kNotEnoughFinderPattern,
		kVersionNotSupported,
		kFormatNotDetected
	};

	const char* whatStr[4] = {
		"File not readable",
		"Unable to detect Finder Pattern",
		"Qr Code Size / Version not supported",
		"Format not detected",
	};
	const Cause cause_;

	QrCodeException(Cause c) : cause_(c)
	{
	}

	const char* what() const noexcept override
	{
		return whatStr[static_cast<int>(cause_)];
	}
};

#endif /* QRCODEEXCEPTION_H_ */
