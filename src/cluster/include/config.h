//the   TCfgFileParser   class.
//用来解析一个基于`section`的配置文件
//配置文件**不允许**有同名section,   不同的section中可以有同名的`key`
/*   样例：
#section   begin
[section_1]
key1   =   value1;   #   comment
key2   =   value2;

[section_2]
key1   =   value3;   #   comment
key2   =   value4;

*/
#ifndef _CONFIG_H
#define   _CONFIG_H

#if   _MSC_VER   >   1000
#pragma   once
#endif   //   _MSC_VER   >   1000

#include   <stdlib.h>
#include   <stdio.h>

#ifdef   WIN32
#pragma   warning(disable:4786)
#pragma   warning(disable:4503)
#endif

#include   <map>
#include   <list>
#include   <vector>
#include   <fstream>
#include   <string>
#include   <iostream>

class   TCfgFileParser
{
public:
	TCfgFileParser(
		char   chSectionBMark   =   '[',
		char   chSectionEMark   =   ']',
		char   chRecordEMark   =   ';',   //record   end-mark
		char   chCommentMark   =   '#'
		)
		: m_chSectionBMark(chSectionBMark), m_chSectionEMark(chSectionEMark),
		m_chRecordEMark(chRecordEMark),   m_chCommentMark(chCommentMark)
	{
	}

	virtual   ~TCfgFileParser()   {}

	typedef   std::map<std::string,   std::string>   SECTION_CONTENT;
	typedef   std::map<std::string,   SECTION_CONTENT   >   FILE_CONTENT;

private:
	enum{	  SYNTAX_ERROR = 0x100,  };

public:
	const   char*   getErrorString(int   err);
	int   parseFile(const char* lpszFilename);
	int   sections()   const   {   return   m_content.size();   }

	FILE_CONTENT::const_iterator sectionBegin()   const   {   return   m_content.begin();   }
	FILE_CONTENT::const_iterator sectionEnd()   const   {   return   m_content.end();   }

	//return   empty-string   if   no   corresponding   value   presented.

	std::string   getValue(const std::string& strSection, const std::string& strKey) const;
	std::string   getValue(const std::string& strSection, const int num,const std::string& strKey) const;
	void   printContent();
	int    getKeyNum();
	FILE_CONTENT   m_content;
protected:
	bool   extractSection(char*   line, std::string& strSection);
	bool   extractKeyValue(char* line, std::pair<std::string, std::string>& pairKeyValue);

private:	

	char   m_chSectionBMark;
	char   m_chSectionEMark;
	char   m_chRecordEMark;
	char   m_chCommentMark;
	int     m_error_line;

};

#endif

