#ifndef MDFN_FILE_H
#define MDFN_FILE_H

#include <string>

#define MDFNFILE_EC_NOTFOUND	1
#define MDFNFILE_EC_OTHER	2

class MDFNFILE
{
	public:

	MDFNFILE();
	// WIP constructors:
	MDFNFILE(const char *path, const void *known_ext, const char *purpose = NULL);

	~MDFNFILE();

	bool Open(const char *path, const void *known_ext, const char *purpose = NULL, const bool suppress_notfound_pe = FALSE);
	INLINE bool Open(const std::string &path, const void *known_ext, const char *purpose = NULL, const bool suppress_notfound_pe = FALSE)
	{
	 return(Open(path.c_str(), known_ext, purpose, suppress_notfound_pe));
	}

	bool Close(void);

   uint8 *f_data;
   int64 f_size;
   char *f_ext;

	private:

        int64 location;

	bool MakeMemWrapAndClose(void *tz);
};

#endif
