#pragma once

#include <string>

#include "webserver/HtmlTagInput.h"
#include "webserver/HtmlTagJavascript.h"

namespace webserver
{
	class HtmlTagInputSlider : public HtmlTag
	{
		public:
			HtmlTagInputSlider(const std::string& name, const unsigned int min, const unsigned int max, const unsigned int value = 0);

			virtual HtmlTag AddAttribute(const std::string& name, const std::string& value)
			{
				childTags[0].AddAttribute(name, value);
				return *this;
			}
	};
};
