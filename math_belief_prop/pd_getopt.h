/* ::[[ @(#) getopt.c 1.5 89/03/11 05:40:23 ]]:: */
#ifndef LINT
static const char Id[] = "$Id: getopt.c,v 1.2 2009/09/01 00:41:59 tom Exp $";
#endif

/*
 * Here's something you've all been waiting for:  the AT&T public domain
 * source for getopt(3).  It is the code which was given out at the 1985
 * UNIFORUM conference in Dallas.  I obtained it by electronic mail
 * directly from AT&T.  The people there assure me that it is indeed
 * in the public domain.
 *
 * There is no manual page.  That is because the one they gave out at
 * UNIFORUM was slightly different from the current System V Release 2
 * manual page.  The difference apparently involved a note about the
 * famous rules 5 and 6, recommending using white space between an option
 * and its first argument, and not grouping options that have arguments.
 * Getopt itself is currently lenient about both of these things White
 * space is allowed, but not mandatory, and the last option in a group can
 * have an argument.  That particular version of the man page evidently
 * has no official existence, and my source at AT&T did not send a copy.
 * The current SVR2 man page reflects the actual behavor of this getopt.
 * However, I am not about to post a copy of anything licensed by AT&T.
 */

#include <stdio.h>
#include <string.h>

#define PD_GETOPT_ERR(szz,czz) if(opterr){fprintf(stderr,"%s%s%c\n",argv[0],szz,czz);}

int opterr = 1;
int optind = 1;
int optopt;
char *optarg;

int pd_getopt(int argc, char **argv, const char *opts) {
  static int sp = 1;
  register int c;
  register const char *cp;

  if (sp == 1) {
    if ((optind >= argc) ||
        (argv[optind][0] != '-') ||
        (argv[optind][1] == '\0')) {
      return (EOF);
    }
    else if (strcmp(argv[optind], "--") == 0) {
      optind++;
      return (EOF);
    }
  }
  optopt = c = argv[optind][sp];

  if ((c == ':') ||
      ((cp = strchr(opts, c)) == NULL)) {
    PD_GETOPT_ERR(": illegal option -- ", c);
    if (argv[optind][++sp] == '\0') {
      optind++;
      sp = 1;
    }
    return ('?');
  }

  if (*++cp == ':') {

    if (argv[optind][sp + 1] != '\0') {
      optarg = &argv[optind++][sp + 1];
    }

    else if (++optind >= argc) {
      PD_GETOPT_ERR(": option requires an argument -- ", c);
      sp = 1;
      return ('?');
    }

    else {
      optarg = argv[optind++];
    }
    sp = 1;
  }
  else {
    if (argv[optind][++sp] == '\0') {
      sp = 1;
      optind++;
    }
    optarg = NULL;
  }
  return (c);
}

