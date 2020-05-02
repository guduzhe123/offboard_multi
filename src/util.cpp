/*
 * =====================================================================================
 *
 *       Filename:  util.c
 *
 *    Description:  This file implements the util functions
 *
 *        Version:  1.0
 *        Created:  01/09/2017
 *       Revision:  01/20/2017
 *
 *         Author:  suhang refer wenfang`s util.c
 *          Email:  wf.lin@clobotics.com
 *        Company:  Clobotics
 *
 * =====================================================================================
 */


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <getopt.h>
#include <assert.h>
#include <fcntl.h>
#include <sys/resource.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <syslog.h>
#include <errno.h>
#include <ctype.h>
#include <iostream>

#include "util.h"

static bool	daemonized = FALSE;
static int	sock_remote = -1;
static FILE	*fp = NULL;
static char	id_name[ID_MAX_LENGTH];
static char	id_number = -1;
static char	log_prog[ID_MAX_LENGTH];
static uint8_t	log_seq = 0;
static struct sockaddr_in	sa_remote;
char LogFile[256]={0};

void util_set_id (const char *name, int number, const char *prog)
{
	if (strlen (prog) < ID_MAX_LENGTH && strlen(name) < ID_MAX_LENGTH && number > 0) {
		strcpy (log_prog, prog);
		strcpy (id_name, name);
		id_number = number;
	}
}

void util_add_remote (int sock, struct sockaddr_in sa)
{
	if (sock >= 0) {
		sock_remote = sock;
		sa_remote = sa;
	}
}

void util_log(const char *fmt, ...)
{
	va_list	ap;
	FILE	*log;
	char	buf[MSG_MAX_SIZE];
	int 	header_len, buf_len;
	time_t 	t;
	char 	time_str[50];
	struct tm *tmp;

	log = daemonized ? fp : stdout;
	if (log == NULL) return;

	t = time(NULL);
	tmp = localtime(&t);
	buf[0] = '\0';
	if (tmp == NULL)
		return;
	if (strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", tmp) == 0) 
		return;
	if (id_number > 0) sprintf (buf, "[%s@%s#%d:%3d] ", log_prog, id_name, id_number, log_seq++);
	strcat(buf, time_str);
	strcat(buf, " => ");
	header_len = strlen(buf);
	va_start(ap, fmt);
	vsnprintf(&buf[header_len], MSG_MAX_SIZE - header_len, fmt, ap);
	buf_len = strlen(buf);
	buf[buf_len]='\n';
	buf[buf_len+1]='\0';
	fputs(buf, log);
	fflush(log);		/* flushes all stdio output streams */
	if (sock_remote != -1) {
		sendto (sock_remote, buf, strlen (buf), 0, (struct sockaddr *) (&sa_remote), sizeof(struct sockaddr_in));
	}
	va_end(ap);
}

void util_daemonize ()
{
	int	i, fd_stdout, fd_stderr, fd_stdin;
	int	res;
	int fd_log;
	pid_t	pid;
	struct rlimit	rl;
	struct sigaction sa;
    char logfile[256]={0};
    char file[256]={0};
	time_t 	t;
	char 	time_str[50];
	struct tm *tmp;
	umask (0);
	
	t = time(NULL);
	tmp = localtime(&t);
	printf("util_daemonize --- \n");
	if (tmp == NULL)
		return;
	if (strftime(time_str, sizeof(time_str), "%Y-%m-%d_%H-%M-%S", tmp) == 0) 
		return;
	printf("time_str=%s\n", time_str);
    auto homedir = std::string(getenv("HOME"));
    sprintf(file, (homedir + "/.ros/log_%s").c_str(), time_str);
    int ret = mkdir(file,  S_IRWXU | S_IRWXG | S_IRWXO);

    sprintf(logfile, "%s/log_%s.txt", file, time_str);
	res = getrlimit (RLIMIT_NOFILE, &rl);
	if (res < 0) {
		util_log ("util_daemonize() -> getrlimit(), %s\n", strerror(errno));
		exit(EXIT_FAILURE);
	}
    strcpy(LogFile, file);
#if 0
	pid = fork();
	if (pid < 0) {
		util_log ("util_daemonize() -> fork(), %s\n", strerror(errno));
		exit(EXIT_FAILURE);
	}
	else if (pid != 0) {// parent
		exit (EXIT_SUCCESS);
	}
	res = setsid ();
	if (res < 0) {
		util_log ("util_daemonize() -> setsid(), %s\n", strerror(errno));
		exit(EXIT_FAILURE);
	}
	sa.sa_handler = SIG_IGN;
	sigemptyset (&sa.sa_mask);
	sa.sa_flags = 0;
	res = sigaction(SIGHUP, &sa, NULL);
	if (res < 0) {
		util_log ("util_daemonize() -> sigaction()\n");
		exit(EXIT_FAILURE);
	}
	pid = fork();
	if (pid < 0) {
		util_log ("util_daemonize() -> fork(), %s\n", strerror(errno));
		exit(EXIT_FAILURE);
	}
	else if (pid != 0) {// parent
		exit (EXIT_SUCCESS);
	}
	res = chdir("/");
	if (res < 0) {
		util_log ("util_daemonize() -> chdir(), %s\n", strerror(errno));
		exit(EXIT_FAILURE);
	}
	if (rl.rlim_max == RLIM_INFINITY)
		rl.rlim_max = 1024;
	for (i = 0; i < rl.rlim_max; i++) 
		close (i);
#endif	
	if (logfile != NULL && logfile[0] != '0') {
		fp = fopen (logfile, "a+");
		if (fp == NULL) {
			util_log ("parse_port_configuration() -> fopen(), %s\n", strerror(errno));
			exit (EXIT_FAILURE);
		}
	}
	daemonized = TRUE;
#if 1	
	fd_log = fileno(fp);
	if (	dup2(fd_log,STDOUT_FILENO) == -1)
	{
		util_log("dup stdout error");
	}
	
	if(dup2(fd_log,STDERR_FILENO) == -1)
	{
		util_log("dup stderr error");
	}
#else
	fd_stdin = open ("/dev/null", O_RDWR);
	fd_stdout = dup(0);
	fd_stderr = dup(0);

	if (fd_stdin != 1 || fd_stdout != 2 || fd_stderr != 3)
		util_log ("util_daemonize(), unexpected file descriptors %d, %d, %d\n", fd_stdin, fd_stdout, fd_stderr);

#endif
	util_log ("Daemonized, pid = %d\n", getpid());
}

void util_SetLogFile(char logfile[256]) {
    strcpy(logfile, LogFile);
}


