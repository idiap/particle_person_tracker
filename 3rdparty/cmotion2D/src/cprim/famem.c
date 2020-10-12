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
  
 DESCRIPTION	: Le fichier (famem = Float Array MEMory) contient les
		  procedures de gestion en memoire des tableaux de reels
		  simple precision monodimensionnels.

*/


#include	<memory.h>

#include	"famem.h"


/*
 * PROCEDURE	: copy_float
 *
 * INPUT	:
 * src		Tableau de reels simple precision source.
 *
 * OUTPUT	:
 * dst		Tableau de reels simple precision destination.
 *
 * INPUT	:
 * n		Nombre d'elements du tableau "src" a copier.
 *
 * DESCRIPTION	:
 * La procedure copie "n" reels simple precision du tableau "src" dans le
 * tableau "dst".
 *
 * HISTORIQUE	:
 * 1.00 - 17/12/91 - Original.
 */
void	copy_float (const float *src, float *dst, size_t n)
{
	const size_t	lsize =	4;

	const float	*send = src + n;	/* borne de "src"	*/

	if (n >= lsize) {	/* boucle deroulee lsize fois	*/
		send -= lsize - 1;
		for (; src < send; src += lsize, dst += lsize) {
			dst[0] = src[0];
			dst[1] = src[1];
			dst[2] = src[2];
			dst[3] = src[3];
		}
		send += lsize - 1;
	}
	for (; src < send; *dst++ = *src++);
}


/*
 * PROCEDURE	: copyinc_float
 *
 * INPUT	:
 * src		Tableau de reels simple precision source.
 *
 * OUTPUT	:
 * dst		Tableau de reels simple precision destination.
 *
 * INPUT	:
 * n		Nombre d'elements du tableau "src" a copier.
 * incsrc	Increment du tableau "src".
 * incdst	Increment du tableau "dst".
 *
 * DESCRIPTION	:
 * La procedure copie "n" reels simple precision du tableau "src" dans le
 * tableau "dst". Le parcourt des "n" reels simple precision source s'effectue
 * a partir du pointeur "src" par increment de "incsrc". Le parcourt des "n" reels
 * simple precision destination s'effectue a partir du pointeur "dst" par
 * increment de "incdst".
 *
 * HISTORIQUE	:
 * 1.00 - 17/12/91 - Original.
 */
void	copyinc_float (const float *src, float *dst, size_t n,
		int incsrc, int incdst)
{
	const size_t	lsize =	4;

	register size_t	i = 0;	/* ordre	*/

	if (n >= lsize) {	/* boucle deroulee lsize fois	*/
		n -= lsize - 1;
		for (; i < n; i += lsize) {
			*dst = *src; src += incsrc; dst += incdst;
			*dst = *src; src += incsrc; dst += incdst;
			*dst = *src; src += incsrc; dst += incdst;
			*dst = *src; src += incsrc; dst += incdst;
		}
		n += lsize - 1;
	}
	for (; i < n; i++) {
		*dst = *src; src += incsrc; dst += incdst;
	}
}


/*
 * PROCEDURE	: set_float
 *
 * OUTPUT	:
 * buf		Tableau de reels simple precision a initialiser.
 *
 * INPUT	:
 * v		Valeur d'initialisation du tableau.
 * n		Nombre d'elements du tableau "buf" a initialiser.
 *
 * DESCRIPTION	:
 * La procedure initialise "n" reels simple precision du tableau "buf" par la
 * valeur "v".
 *
 * HISTORIQUE	:
 * 1.00 - 17/12/91 - Original.
 */
void	set_float (float *buf, float v, size_t n)
{
	const size_t	lsize =	4;

	register float	*bend = buf + n;	/* borne de "buf"	*/
	register float	t = v;			/* optimise ?		*/

	if (n >= lsize) {	/* boucle deroulee lsize fois	*/
		bend -= lsize - 1;
		for (; buf < bend; buf += lsize) {
			buf[0] = t;
			buf[1] = t;
			buf[2] = t;
			buf[3] = t;
		}
		bend += lsize - 1;
	}
	for (; buf < bend; *buf++ = t);
}


/*
 * PROCEDURE	: setinc_float
 *
 * OUTPUT	:
 * buf		Tableau de reels simple precision a initialiser.
 *
 * INPUT	:
 * v		Valeur d'initialisation du tableau.
 * n		Nombre d'elements du tableau "buf" a initialiser.
 * inc		Increment du tableau "buf".
 *
 * DESCRIPTION	:
 * La procedure initialise "n" reels simple precision du tableau "buf" par la
 * valeur "v". Le parcourt des "n" reels simple precision s'effectue a partir du
 * pointeur "buf" par increment de "inc".
 *
 * HISTORIQUE	:
 * 1.00 - 17/12/91 - Original.
 */
void	setinc_float (float *buf, float v, size_t n, int inc)
{
	const size_t	lsize =	4;

	register float	t = v;	/* optimise ?	*/
	register size_t	i = 0;	/* ordre	*/

	if (n >= lsize) {	/* boucle deroulee lsize fois	*/
		n -= lsize - 1;
		for (; i < n; i += lsize) {
			*buf = t; buf += inc;
			*buf = t; buf += inc;
			*buf = t; buf += inc;
			*buf = t; buf += inc;
		}
		n += lsize - 1;
	}
	for (; i < n; i++) {
		*buf = t; buf += inc;
	}
}


#ifdef	FAMEM_MAIN

#define	BASEN		1000000

#include	<stdio.h>
#include	<sys/param.h>
#include	<sys/types.h>
#include	<sys/times.h>

/*
 * Objets associes au temps CPU :
 * CLICKSPERSEC	Frequence en Hertz du compteur de l'horloge.
 */
#define	CLICKSPERSEC	HZ

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


static	float	dst[BUFSIZE];
static	float	src[BUFSIZE];

static	char	prog_name[] = "famem";


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
long	jobclicks ()
{
#ifdef	unix
	struct	tms	buffer;

	times (&buffer);
	return ((long) (buffer.tms_utime + buffer.tms_stime));
#endif	/* unix	*/

#ifdef	msdos
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
 * 1.00 - 06/07/95 - Original.
 */
static	void	print_usage ()
{
	static	char	usage[] = "Usage: %s basen\n";

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
 * 1.00 - 06/07/95 - Original.
 */
void	main (argc, argv, envp)
int	argc;
char	*argv[], *envp[];
{
	extern	long	atol (/* const char * */);

	long	basen;	/* base d'iterations	*/

	if (argc != 2) {
		print_usage ();
		exit (1);
	}

	basen = atol (argv[1]);

	LOOP(copy_float (src, dst, BUFSIZE), basen);
	LOOP(set_float (src, 0.F, BUFSIZE), basen);
	LOOP(setinc_float (src, 0.F, BUFSIZE, 1), basen);

	exit (0);
}

#endif	/* FAMEM_MAIN	*/

