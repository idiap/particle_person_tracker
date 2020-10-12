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
  
 DESCRIPTION	: Le fichier (faarith = Float Array ARITHmetic) contient les
		  procedures arithmetiques operant sur les tableaux de reels
		  simple precision monodimensionnel.

*/


#include	<math.h>

#include	"faarith.h"


/*
 * PROCEDURE	: add_float
 *
 * OUTPUT	:
 * a		Tableau de reels simple precision operande gauche et droit.
 *
 * INPUT	:
 * b		Tableau de reels simple precision operande droit.
 * n		Nombre d'elements dans les tableaux "a" et "b".
 *
 * DESCRIPTION	:
 * La procedure additionne au tableau de reels simple precision "a" le tableau
 * de reels simple precision "b" : a[i] += b[i].
 *
 * HISTORIQUE	:
 * 1.00 - 24/07/95 - Original.
 */
void	add_float (float *a, const float *b, size_t n)
{
	const size_t	lsize = 4;

	register float	*aend = a + n;	/* borne de "a"	*/

	if (n >= lsize) {	/* boucle deroulee lsize fois	*/
		aend -= lsize - 1;
		for (; a < aend; a += lsize, b += lsize) {
			a[0] += b[0];
			a[1] += b[1];
			a[2] += b[2];
			a[3] += b[3];
		}
		aend += lsize - 1;
	}
	for (; a < aend; *a++ += *b++);
}

/*
 * PROCEDURE	: addconst_float
 *
 * OUTPUT	:
 * a		Tableau de floats operande gauche et droit.
 *
 * INPUT	:
 * v		Valeur d'addition operande droit.
 * n		Nombre d'elements dans le tableau "a".
 *
 * DESCRIPTION	:
 * La procedure additionne aux "n" reels simple precision du tableau "a" la
 * valeur v.
 *
 * HISTORIQUE	:
 * 1.00 - 26/07/95 - Original.
 */
void	addconst_float (register float *a, float v, size_t n)
{
	const size_t	lsize = 4;

	register float	*aend = a + n;	/* borne de "a"	*/
	register float	t = v;		/* optimise ?	*/

	if (n >= lsize) {	/* boucle deroulee lsize fois	*/
		aend -= lsize - 1;
		for (; a < aend; a += lsize) {
			a[0] += t;
			a[1] += t;
			a[2] += t;
			a[3] += t;
		}
		aend += lsize - 1;
	}
	for (; a < aend; *a++ += t);
}

/*
 * PROCEDURE	: addinc_float
 *
 * OUTPUT	:
 * a		Tableau de reels simple precision operande gauche et droit.
 *
 * INPUT	:
 * b		Tableau de reels simple precision operande droit.
 * n		Nombre d'elements dans les tableaux "a" et "b".
 * inca		Increment du pointeur "a".
 * incb		Increment du pointeur "b".
 *
 * DESCRIPTION	:
 * La procedure additionne au tableau de reels simple precision "a" le tableau
 * de reels simple precision "b" : a[i] += b[i].
 * Le parcourt des "n" reels simple precision s'effectue a partir du pointeur "a"
 * par increment de "inca". Le parcourt des "n" reels simple precision s'effectue
 * a partir du pointeur "b" increment de "incb".
 *
 * HISTORIQUE	:
 * 1.00 - 24/07/95 - Original.
 */
void	addinc_float (float *a, const float *b, size_t n, int inca, int incb)
{
	const size_t	lsize = 4;

	register size_t	i = 0;	/* ordre	*/

	if (n >= lsize) {	/* boucle deroulee lsize fois	*/
		n -= lsize - 1;
		for (; i < n; i += lsize) {
			*a += *b; a += inca; b += incb;
			*a += *b; a += inca; b += incb;
			*a += *b; a += inca; b += incb;
			*a += *b; a += inca; b += incb;
		}
		n += lsize - 1;
	}
	for (; i < n; i++) {
		*a += *b; a += inca; b += incb;
	}

}

/*
 * PROCEDURE	: multconst_float
 *
 * OUTPUT	:
 * a		Tableau de floats operande gauche et droit.
 *
 * INPUT	:
 * v		Valeur de multiplication operande droit.
 * n		Nombre d'elements dans le tableau "a".
 *
 * DESCRIPTION	:
 * La procedure multiplie les "n" reels simple precision du tableau "a" par la
 * valeur v.
 *
 * HISTORIQUE	:
 * 1.00 - 04/01/95 - Original.
 */
void	multconst_float (register float	*a, float v, size_t n)
{
	const size_t	lsize = 4;

	register float	*aend = a + n;	/* borne de "a"	*/
	register float	t = v;		/* optimise ?	*/

	if (n >= lsize) {	/* boucle deroulee lsize fois	*/
		aend -= lsize - 1;
		for (; a < aend; a += lsize) {
			a[0] *= t;
			a[1] *= t;
			a[2] *= t;
			a[3] *= t;
		}
		aend += lsize - 1;
	}
	for (; a < aend; *a++ *= t);
}

/*
 * PROCEDURE	: multiply_float
 *
 * OUTPUT	:
 * a		Tableau de reels simple precision operande gauche et droit.
 *
 * INPUT	:
 * b		Tableau de reels simple precision operande droit.
 * n		Nombre d'elements dans les tableaux "a" et "b".
 *
 * DESCRIPTION	:
 * La procedure multiplie le tableaux de reels simple precision "a" par le
 * tableau de reels simple precision "b" : a[i] *= b[i].
 *
 * HISTORIQUE	:
 * 1.00 - 24/07/95 - Original.
 */
void	multiply_float (float *a, const float *b, size_t n)
{
	const size_t	lsize = 4;

	float		*aend = a + n;	/* borne de "a"	*/

	if (n >= lsize) {	/* boucle deroulee lsize fois	*/
		aend -= lsize - 1;
		for (; a < aend; a += lsize, b += lsize) {
			a[0] *= b[0];
			a[1] *= b[1];
			a[2] *= b[2];
			a[3] *= b[3];
		}
		aend += lsize - 1;
	}
	for (; a < aend; *a++ *= *b++);
}

/*
 * PROCEDURE	: subtract_float
 *
 * OUTPUT	:
 * a		Tableau de reels simple precision operande gauche et droit.
 *
 * INPUT	:
 * b		Tableau de reels simple precision operande droit.
 * n		Nombre d'elements dans les tableaux "a" et "b".
 *
 * DESCRIPTION	:
 * La procedure soustrait du tableau de reels simple precision "a" le tableau
 * de reels simple precision "b" : a[i] -= b[i].
 *
 * HISTORIQUE	:
 * 1.00 - 25/07/95 - Original.
 */
void	subtract_float (register float *a, register float *b, size_t n)
{
	const size_t	lsize = 4;

	register float	*aend = a + n;	/* borne de "a"	*/

	if (n >= lsize) {	/* boucle deroulee lsize fois	*/
		aend -= lsize - 1;
		for (; a < aend; a += lsize, b += lsize) {
			a[0] -= b[0];
			a[1] -= b[1];
			a[2] -= b[2];
			a[3] -= b[3];
		}
		aend += lsize - 1;
	}
	for (; a < aend; *a++ -= *b++);
}

/*
 * PROCEDURE	: subinc_float
 *
 * OUTPUT	:
 * a		Tableau de reels simple precision operande gauche et droit.
 *
 * INPUT	:
 * b		Tableau de reels simple precision operande droit.
 * n		Nombre d'elements dans les tableaux "a" et "b".
 * inca		Increment du pointeur "a".
 * incb		Increment du pointeur "b".
 *
 * DESCRIPTION	:
 * La procedure soustrait du tableau de reels simple precision "a" le tableau
 * de reels simple precision "b" : a[i] -= b[i].
 * Le parcourt des "n" reels simple precision s'effectue a partir du pointeur "a"
 * par increment de "inca". Le parcourt des "n" reels simple precision s'effectue
 * a partir du pointeur "b" increment de "incb".
 *
 * HISTORIQUE	:
 * 1.00 - 24/07/95 - Original.
 */
void	subinc_float (float *a, const float *b, size_t n, int inca, int incb)
{
	const size_t	lsize = 4;

	register size_t	i = 0;	/* ordre	*/

	if (n >= lsize) {	/* boucle deroulee lsize fois	*/
		n -= lsize - 1;
		for (; i < n; i += lsize) {
			*a -= *b; a += inca; b += incb;
			*a -= *b; a += inca; b += incb;
			*a -= *b; a += inca; b += incb;
			*a -= *b; a += inca; b += incb;
		}
		n += lsize - 1;
	}
	for (; i < n; i++) {
		*a -= *b; a += inca; b += incb;
	}

}


#ifdef	FAARITH_MAIN

#define	BASEN		1000000

#include	<stdio.h>
#ifdef	_MSDOS
#include	<time.h>
#else
#include	<sys/param.h>
#include	<sys/types.h>
#include	<sys/times.h>
#endif	/* ! _MSDOS	*/

/*
 * Objets associes au temps CPU :
 * CLICKSPERSEC	Frequence en Hertz du compteur de l'horloge.
 */
#ifdef	_MSDOS
#define	CLICKSPERSEC	CLOCKS_PER_SEC
#else
#define	CLICKSPERSEC	HZ
#endif	/* ! _MSDOS	*/

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
	((float) (_loopend-_loopstart) * 1e6) / (float) (CLICKSPERSEC * _n));\
}


#define	BUFSIZE	1000	/* taille des tableaux	*/


static	float	a[BUFSIZE];
static	float	b[BUFSIZE];

static	char	prog_name[] = "faarith";


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
 * 1.00 - 06/07/95 - Original.
 */
static	void	print_usage ()
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
 * 1.00 - 06/07/95 - Original.
 */
void	main (int argc, char *argv[])
{
	extern	long	atol (const char *);

	long	basen;	/* base d'iterations	*/
	size_t	i;

	if (argc != 2) {
		print_usage ();
		exit (1);
	}

	basen = atol (argv[1]);

	/* initailise les tableaux	*/
	for (i = 0; i < BUFSIZE; i++)
		a[i] = (float) rand ();
	for (i = 0; i < BUFSIZE; i++)
		b[i] = 1.F;


	LOOP(add_float (a, b, BUFSIZE), basen);
	LOOP(addinc_float (a, b, BUFSIZE, 1, 1), basen);
	LOOP(subtract_float (a, b, BUFSIZE), basen);
	LOOP(subinc_float (a, b, BUFSIZE, 1, 1), basen);
	LOOP(multiply_float (a, b, BUFSIZE), basen);
#if	0
	LOOP(addconst_float (a, (float) 1.0, BUFSIZE), basen);
	LOOP(dot_float (a, b, BUFSIZE), basen);
	LOOP(dotinc_float (a, b, BUFSIZE, 1, -1), basen);
	LOOP(multconst_float (a, (float) 1.0, BUFSIZE), basen);
	LOOP(multiply_float (a, b, BUFSIZE), basen);
#endif

	exit (0);
}

#endif	/* FAARITH_MAIN	*/

