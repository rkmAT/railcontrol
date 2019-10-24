/*
RailControl - Model Railway Control Software

Copyright (c) 2017-2019 Dominik (Teddy) Mahrer - www.railcontrol.org

RailControl is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 3, or (at your option) any
later version.

RailControl is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with RailControl; see the file LICENCE. If not see
<http://www.gnu.org/licenses/>.
*/

#pragma once

#include <string>

#include "WebServer/HtmlTag.h"
#include "WebServer/HtmlTagLabel.h"
#include "WebServer/HtmlTagSelect.h"

namespace WebServer
{
	class HtmlTagSelectWithLabel : public HtmlTag
	{
		public:
			HtmlTagSelectWithLabel(const std::string& name, const std::string& label, const std::map<std::string,std::string>& options, const std::string& defaultValue = "")
			:	HtmlTag()
			{
				AddChildTag(HtmlTagLabel(label, "s_" + name));
				AddChildTag(HtmlTagSelect(name, options, defaultValue));
			}

			template<typename T> HtmlTagSelectWithLabel(const std::string& name, const std::string& label, const std::map<std::string,T>& options, const int defaultValue = 0)
			:	HtmlTag()
			{
				AddChildTag(HtmlTagLabel(label, "s_" + name));
				AddChildTag(HtmlTagSelect(name, options, defaultValue));
			}

			virtual HtmlTag AddAttribute(const std::string& name, const std::string& value) override
			{
				childTags[1].AddAttribute(name, value);
				return *this;
			}

			virtual HtmlTag AddClass(const std::string& _class) override
			{
				childTags[1].AddClass(_class);
				return *this;
			}
	};
};
