/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/
/*
  
 DESCRIPTION	: Le fichier (acast = Array CAST) contient les procedures
		  de conversions de types de tableaux monodimensionnels.

*/


#include	"acast.h"


#define	CAT(a,b)	a ## b
#define	XCAT(a,b)	CAT(a,b)

#define	CAST(T1,T2)	XCAT(cast_, XCAT(T1, XCAT(_, T2)))
#define	ACAST(T1,T2)	\
void	CAST(T1,T2) (const T1 *src, T2 *dst, size_t n)\
{\
	const size_t	lsize = 4;\
\
	const T1	*send = src + n;\
\
	if (n >= lsize) {\
		send -= lsize - 1;\
		for (; src < send; src += lsize, dst += lsize) {\
			dst[0] = (T2) src[0];\
			dst[1] = (T2) src[1];\
			dst[2] = (T2) src[2];\
			dst[3] = (T2) src[3];\
		}\
		send += lsize - 1;\
	}\
\
	for (; src < send; *dst++ = (T2) *src++);\
}


#define	CAST_(T1, T2)		XCAT(cast_, XCAT(T1, XCAT(_, T2)))
#define	CASTINC_(T1, T2)	XCAT(castinc_, XCAT(T1, XCAT(_, T2)))

/*
 * MACRO	: CASTINC
 *
 * INPUT	:
 * src		Tableau de type T1 source.
 *
 * OUTPUT	:
 * dst		Tableau de type T2 destination.
 *
 * INPUT	:
 * n		Nombre d'elements du tableau "src" a convertir.
 * incsrc	Increment du tableau "src".
 * incdst	Increment du tableau "dst".
 *
 * DESCRIPTION	:
 * La macro-instruction convertit "n" elements de type "T1" du tableau "src"
 * en "n" elements de type "T2" dans le tableau "dst". La procedure balaye le
 * tableau "src" par increment de "incsrc" et le tableau "dst" et par
 * increment de "incdst".
 *
 * HISTORIQUE	:
 * 1.00 - 01/10/95 - Original.
 */
#define	CASTINC(T1,T2)	\
void	CASTINC_(T1, T2) (const T1 *src, T2 *dst, size_t n,\
		int incsrc, int incdst)\
{\
	const size_t	lsize = 4;\
\
	register size_t	i = 0;\
\
	if (n >= lsize) {\
		n -= lsize - 1;\
		for (; i < n; i += lsize) {\
			*dst = (T2) *src; src += incsrc; dst += incdst;\
			*dst = (T2) *src; src += incsrc; dst += incdst;\
			*dst = (T2) *src; src += incsrc; dst += incdst;\
			*dst = (T2) *src; src += incsrc; dst += incdst;\
		}\
		n += lsize - 1;\
	}\
\
	for (; i < n; i++) {\
		*dst = (T2) *src; src += incsrc; dst += incdst;\
	}\
}



ACAST(double,float)
ACAST(float,double)
ACAST(float,short)
ACAST(short,float)
ACAST(short,uchar)
ACAST(uchar,short)
ACAST(char,short)
ACAST(uchar,uint)


CASTINC(short,float)

#ifdef	ACAST_MAIN

#define	BASEN		1000000

#include	<stdio.h>
#ifdef	_MSDOS
#include	<time.h>
#else
#include	<sys/param.h>
#include	<sys/types.h>
#include	<sys/times.h>
#endif	/* _MSDOS	*/


/*
 * Objets associes au temps CPU :
 * CLICKSPERSEC	Frequence en Hertz du compteur de l'horloge.
 */
#ifdef	_MSDOS
#define	CLICKSPERSEC	CLOCKS_PER_SEC
#else
#define	CLICKSPERSEC	HZ
#endif	/* _MSDOS	*/

/*
 * Macros principales pour les tests.
 */
#define	quoted(TEXT)	#TEXT

#define	LOOP(CODE, N)	\
{\
	long	_i,		/* indice de la boucle		*/\
		_n,		/* taille de la boucle		*/\
		_loopstart,	/* top du debut de la boucle	*/\
		_loopend;	/* top de fin   de la boucle	*/\
\
	_n = (long) (N);\
	printf ("%-64s%ld\t", quoted(CODE), _n);\
	_loopstart = jobclicks ();\
	for (_i = 0L; _i < _n; _i++) { CODE; }\
	_loopend = jobclicks ();\
	printf ("%6.0f\n", \
	((float) (_loopend - _loopstart) * 1e6) / (float) (CLICKSPERSEC * _n));\
}


#define	BUFSIZE	1000	/* taille des tableaux	*/


static	double	dbuf[BUFSIZE];
static	float	fbuf[BUFSIZE];
static	short	sbuf[BUFSIZE];
static	uchar	ucbuf[BUFSIZE];

static	char	prog_name[] = "acast";


/*
 * PROCEDURE	: jobclicks
 *
 * DESCRIPTION	:
 * La procedure retourne la somme des cycles horloge utilises par l'utilisateur
 * et par le systeme.
 *
 * RETOUR	:
 * Somme des cycles horloge utilisateur et systeme.
 *
 * HISTORIQUE	:
 * 1.00 - 16/04/91 - Original.
 */
long	jobclicks (void)
{
#ifdef	unix
	struct	tms	buffer;

	times (&buffer);
	return ((long) (buffer.tms_utime + buffer.tms_stime));
#endif	/* unix	*/

#ifdef	_MSDOS
	return ((long) clock ());
#endif	/* msdos	*/
}

/*
 * PROCEDURE	: print_usage
 *
 * GLOBAL	:
 * prog_name	Nom du programme.
 * stderr	Fichier standard d'erreur.
 *
 * DESCRIPTION	:
 * La procedure affiche le mode d'emploi du programme sur le fichier standard
 * d'erreur.
 *
 * HISTORIQUE	:
 * 1.00 - 07/07/95 - Original.
 */
static	void	print_usage (void)
{
	static	char	usage[] = "Usage: %s basen (times in micro-seconds)\n";

	(void) fprintf (stderr, usage, prog_name);
}

/*
 * PROCEDURE	: main
 *
 * GLOBAL	:
 *		Toutes les variables globales.
 *
 * INPUT	:
 * argc		Nombre de parametres   de la ligne de commande du programme.
 * argv		Tableau des parametres de la ligne de commande du programme.
 *		argv[0]		Nom du programme.
 * envp		Tableau des parametres d'environnement du programme.
 *
 * DESCRIPTION	:
 * La procedure teste le fonctionnement et le temps execution des procedures
 * du fichier.
 *
 * RETOUR	:
 * La valeur nulle est retournee si le programme s'execute sans erreur.
 * Sinon une valeur non nulle est retournee.
 *
 * HISTORIQUE	:
 * 1.00 - 07/07/95 - Original.
 */
void	main (int argc, char *argv[])
{
	extern	long	atol (const char *);
	extern	int	rand ();

	long	basen;	/* base d'iterations	*/
	size_t	i;

	if (argc != 2) {
		print_usage ();
		exit (1);
	}
	basen = atol (argv[1]);

	/* teste "double_to_float"	*/
	for (i = 0; i < BUFSIZE; i++)
		dbuf[i] = (double) rand ();
	cast_double_float (dbuf, fbuf, BUFSIZE);
	for (i = 0; i < BUFSIZE; i++) {
		if ((float) dbuf[i] != fbuf[i]) {
			fprintf (stderr, "double_to_float: ?\n");
			break;
		}
	}

	/* teste "float_to_double"	*/
	for (i = 0; i < BUFSIZE; i++)
		fbuf[i] = (float) rand ();
	cast_float_double (fbuf, dbuf, BUFSIZE);
	for (i = 0; i < BUFSIZE; i++) {
		if ((double) fbuf[i] != dbuf[i]) {
			fprintf (stderr, "float_to_double: ?\n");
			break;
		}
	}

	/* teste "short_to_float"	*/
	for (i = 0; i < BUFSIZE; i++)
		sbuf[i] = (short) rand ();
	cast_short_float (sbuf, fbuf, BUFSIZE);
	for (i = 0; i < BUFSIZE; i++) {
		if ((float) sbuf[i] != fbuf[i]) {
			fprintf (stderr, "short_to_float: ?\n");
			break;
		}
	}

	/* teste le temps d'execution	*/
	LOOP(cast_double_float (dbuf, fbuf, BUFSIZE), basen);
	LOOP(cast_float_double (fbuf, dbuf, BUFSIZE), basen);
	LOOP(cast_float_short (fbuf, sbuf, BUFSIZE), basen);
	LOOP(cast_short_float (sbuf, fbuf, BUFSIZE), basen);
	LOOP(cast_short_uchar (sbuf, ucbuf, BUFSIZE), basen);
	LOOP(cast_uchar_short (ucbuf, sbuf, BUFSIZE), basen);

	LOOP(castinc_short_float (sbuf, fbuf, BUFSIZE, 1, 1), basen);

	exit (0);
}

#endif	/* ACAST_MAIN	*/

