// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <sstream>

namespace ORUtils
{

/**
 * \brief An instance of this struct can be used to indicate the failure of a lexical conversion.
 */
struct bad_lexical_cast {};

/**
 * \brief Performs a lexical conversion from the source type to the target type.
 *
 * This is a lightweight replacement for boost::lexical_cast. It's not as
 * sophisticated as that, but it works well enough.
 *
 * \param src               The source value to convert.
 * \return                  The converted value.
 * \throws bad_lexical_cast If the conversion failed.
 */
template <typename Target, typename Source>
Target lexical_cast(const Source& src)
{
	std::stringstream ss;
	ss << src;
	Target target;
	if(!(ss >> target) || !ss.eof()) throw bad_lexical_cast();
	return target;
}

}
