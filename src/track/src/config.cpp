#include   <errno.h>
#include   <assert.h>
#include   <stdlib.h>
#include   <stdio.h>
#include   <string.h>
#include   <memory.h>
#include   "config.h"

static inline bool   istab(int   c)   { return (c == '\t'); }
static inline char*   strltrim(char* str)
{
	while(isspace(*str)   ||   istab(*str))
	{
		++str;
	}
	return   str;
}

static   inline   char*   strrtrim(char*   str)
{
	int   len   =   strlen(str)   -   1;
	while(isspace(str[len])   ||   istab(str[len]))
	{
		str[len--]   =   '\0';
	}
	return   str;
}


int  TCfgFileParser::parseFile(const char *lpszFilename)
{
	using   std::string;
	std::ifstream in(lpszFilename);
	printf("%s\n",lpszFilename);
	if(!in.is_open())
	{
		return errno;
	}
	char   line[1024];
	char*   pline;
	bool   bInSection   =   false;
	SECTION_CONTENT*   pSC   =   NULL;
	string   strSection;
	std::pair<string,   string>   pairKeyValue;

	m_error_line   =   0;
	m_content.clear();
	while(!in.eof())
	{
		in.getline(line,   sizeof(line));
		pline   =   line;
		++m_error_line;
		pline   =   strltrim(pline);
		if(pline[0]   ==   '\0')   continue;   //white   line,   skip
		if(pline[0]   ==   m_chCommentMark)   continue;   //comment   line,   skip
		if(bInSection)
		{
			//is   new-section   begin?
			if(pline[0]   ==   m_chSectionBMark   &&   extractSection(pline,   strSection))
			{
				pSC   =   &m_content[strSection];
			}
			else if(extractKeyValue(pline,pairKeyValue))
			{
				//key-value   pair
				assert(pSC   !=   NULL);
				pSC->insert(pairKeyValue);
			}else
			{
				in.close();
				return   SYNTAX_ERROR;
			}
		}
		else
		{   //NOT   in   section
			//is   a   valid   section?
			if(extractSection(pline,   strSection))
			{
				pSC   =   &m_content[strSection];
				bInSection   =   true;
			}else
			{
				in.close();
				return   SYNTAX_ERROR;
			}
		}
	}

	in.close();
	return   0;
}


std::string   TCfgFileParser::getValue(const   std::string&   strSection,
									   const   std::string&   strKey)   const
{
	FILE_CONTENT::const_iterator   it;
	if((it   =   m_content.find(strSection))   !=   m_content.end())
	{
		SECTION_CONTENT::const_iterator   p;
		const   SECTION_CONTENT&   section   =   (*it).second;//m_content[strSection];
		if((p   =   section.find(strKey))   !=   section.end())
		{
			return   ((*p).second);
		}
	}
	return   "";
	/*
	最简单的是直接：   return   m_content[strSection][strKey];
	注意需要去除   const   修饰。
	但这样会有一些负面作用：如果指定的section或key不存在，则该操作将自动添加相应的空白项。
	*/
}

std::string TCfgFileParser::getValue(const std::string& strSection, const int num,const std::string& strKey)  const
{
	FILE_CONTENT::const_iterator   it;
    char strTempSection[256];
    sprintf(strTempSection,"%s_%d",strSection.c_str(),num);
	if((it   =   m_content.find(strTempSection))   !=   m_content.end())
	{
		SECTION_CONTENT::const_iterator   p;
		const   SECTION_CONTENT&   section   =   (*it).second;//m_content[strSection];
		if((p   =   section.find(strKey))   !=   section.end())
		{
			return   ((*p).second);
		}
	}
	return   "";
}




bool   TCfgFileParser::extractSection(char* line, std::string& strSection)
{
	char*   tmp;
	if(line[0]   ==   m_chSectionBMark)
	{
		if((tmp   =   strchr(++line,   m_chSectionEMark))   !=   NULL)
		{
			*tmp   =   '\0';
			strSection   =   line;
			return   true;
		}
	}

	return   false;
}




bool   TCfgFileParser::extractKeyValue(char*   line,
									   std::pair<std::string, std::string>& pairKeyValue)
{
	char*   tmp;
	if((tmp   =   strrchr(line,m_chRecordEMark))   !=   NULL)
	{
		*tmp   =   '\0';   
		if((tmp   =   strchr(line,   '='))   !=   NULL)
		{
			*tmp++   =   '\0';
			tmp   =   strltrim(tmp);
			tmp   =   strrtrim(tmp);
			line   =   strrtrim(line);

			pairKeyValue.first   =   line;
			pairKeyValue.second   =   tmp;
			return   true;
		}
	}
	return   false;
}

const char* TCfgFileParser::getErrorString(int   err)
{
	static   char   buf[100];
	if(err   ==   SYNTAX_ERROR)
	{
		sprintf(buf,   "configuration   file   format   is   invalid   at   line   %d",   m_error_line);
		return   buf;
	}
	else
	{
		return   strerror(err);
	}
}


void   TCfgFileParser::printContent()
{
	using   std::cout;
	using   std::endl;
	FILE_CONTENT::const_iterator   pf;
	SECTION_CONTENT::const_iterator   ps;
	for(pf   =   m_content.begin();   pf   !=   m_content.end();   ++   pf)
	{
		cout   <<   "section:"   <<   (*pf).first   <<   endl;
		const   SECTION_CONTENT&   sc   =   (*pf).second;
		for(ps   =   sc.begin();   ps   !=   sc.end();   ++ps)
		{
			cout   <<'\t'   <<   (*ps).first   <<   "="   <<   (*ps).second   <<   endl;
		}
	}
}

int   TCfgFileParser::getKeyNum()
{
	FILE_CONTENT::const_iterator   pf;
	SECTION_CONTENT::const_iterator   ps;
	int totalNum = 0;
	for(pf   =   m_content.begin();   pf  !=   m_content.end();   ++pf)
	{
		const   SECTION_CONTENT&   sc   =   (*pf).second;
		for(ps   =   sc.begin();   ps   !=   sc.end();   ++ps)
		{
			totalNum++;
		}
	}
	return totalNum;
}


void TCfgFileParser::SaveFile(const char *lpszFilename)
{
	std::ofstream outfile;
	outfile.open( lpszFilename, std::ios::trunc | std::ios::out );
	using   std::cout;
	using   std::endl;
	FILE_CONTENT::const_iterator   pf;
	SECTION_CONTENT::const_iterator   ps;
	for(pf   =   m_content.begin();   pf   !=   m_content.end();   ++   pf)
	{
		outfile << "["<<(*pf).first<<"]"<<endl;
		const   SECTION_CONTENT&   sc   =   (*pf).second;
		for(ps   =   sc.begin();   ps   !=   sc.end();   ++ps)
		{
			outfile<<(*ps).first<< " = "<<(*ps).second<<";"<<endl;
		}
	}
	outfile.flush();
	outfile.close();
};


