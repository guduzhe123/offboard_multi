/*
 * =====================================================================================
 *
 *       Filename:  util.h
 *
 *    Description:  This file defines useful functions for the program
 *
 *        Version:  1.0
 *        Created:  01/09/2017
 *       Revision:  01/10/2017
 *
 *         Author:  Wenfeng LIN
 *          Email:  wf.lin@clobotics.com
 *        Company:  Clobotics
 *
 * =====================================================================================
 */


#ifndef _UTIL_H_
#define _UTIL_H_

#define MSG_MAX_SIZE		4096
#define PTR_MAX_NUMBER		100
#define ID_MAX_LENGTH		100

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>

#define UM_PI                        3.14159265359
#define mdeg2rad(x)                  (((x) * UM_PI) / 180.f)
#define mrad2deg(x)                  (((x) * 180.f) / UM_PI)
#ifndef DEG2RAD
#define DEG2RAD                      0.01745329252                      
#endif

enum {
	FALSE = 0,
	TRUE
};

typedef struct {
	char **list;
	int count;
} string_list_t;

void util_set_id (const char *name, int number, const char *prog);

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  util_log
 *  Description:  Print log information to standard output
 * =====================================================================================
 *   Parameters:  same as printf
 *      Returns:  none
 * =====================================================================================
 */
void util_log(const char *fmt, ...);

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  util_add_remote
 *  Description:  Add remote server to log destination list
 * =====================================================================================
 *   Parameters:  
 *   		  sock	=> socket used to send log msg
 *   		  sa	=> target address of log msg
 *      Returns:  none
 * =====================================================================================
 */
void util_add_remote (int sock, struct sockaddr_in sa);

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  util_daemonize
 *  Description:  Initialize the current process as a daemon
 * =====================================================================================
 *   Parameters:  none
 *   		  logfile => log file
 *      Returns:  none
 * =====================================================================================
 */
void util_daemonize ();


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  util_string_trim
 *  Description:  Get rid of white spaces from both sides
 * =====================================================================================
 *   Parameters:  
 *   		  buf	=> pointer to buffer, no need to be null-terminated
 *   		  size	=> buffer size
 *      Returns:  
 *      	  pointer to output string which is null-terminated, MUST be free()ed by caller. NULL on failure.
 * =====================================================================================
 */
char *util_string_trim (const char *buf, int size);

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  util_string_split
 *  Description:  Divide string into substrings, substring that contains only white spaces will be discard
 * =====================================================================================
 *   Parameters:  
 *   		  haystack	=> string to be divided
 *   		  needle	=> used to divide haystack
 *   		  at_most	=> maximum number of substring, <= 0 for infinity 
 *      Returns:  
 *      	  string list, MUST be util_string_list_free()ed by caller. list.count = 0 on failure.
 * =====================================================================================
 */
string_list_t util_string_split (const char *haystack, const char *needle, int at_most);

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  util_string_list_free
 *  Description:  Free string list
 * =====================================================================================
 *   Parameters:  
 *   		  list	=> pointer of string list to be freed
 *      Returns:  none
 * =====================================================================================
 */
void util_string_list_free (string_list_t *list);

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  util_string_replace_once
 *  Description:  Replace first substring
 * =====================================================================================
 *   Parameters:  
  *   		  string	=> string to be operated
 *   		  original	=> sub string to be replace 
 *   		  replace	=> replacement
 *      Returns:  
 *      	  Result string, MUST be free()ed by caller. NULL on failure.
 * =====================================================================================
 */
char *util_string_replace_once (const char *string, const char *original, const char *replace);


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  util_realloc
 *  Description:  Allocate memory dynamically
 * =====================================================================================
 *   Parameters:  same as realloc
 *      Returns:  same as realloc
 * =====================================================================================
 */
void *util_realloc (void *ptr, int size);

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  util_strndup
 *  Description:  Duplicate string
 * =====================================================================================
 *   Parameters: 
 *   		  str	=> string to duplicate
 *   		  size	=> number of char to copy, <=0 means the whole string
 *      Returns:  
 *      	  result string
 * =====================================================================================
 */
char *util_strndup (const char *str, int size);

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  util_free
 *  Description:  Free memory
 * =====================================================================================
 *   Parameters:  same as free
 *      Returns:  same as free
 * =====================================================================================
 */
void util_free (void *ptr);


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  util_free_all
 *  Description:  Free all memory in the process
 * =====================================================================================
 *   Parameters:  none
 *      Returns:  none
 * =====================================================================================
 */
void util_free_all (void);


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  util_display_resource
 *  Description:  Display resource
 * =====================================================================================
 *   Parameters:  none
 *      Returns:  none
 * =====================================================================================
 */
void util_display_resource (void);


void util_SetLogFile(char logfile[256]);
#endif
