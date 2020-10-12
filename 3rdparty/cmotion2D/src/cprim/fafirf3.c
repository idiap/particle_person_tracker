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
  
 DESCRIPTION	: Le fichier (fafirf3 = Float Array Finite Impulse Response
		  Filter 3) contient les procedures de filtrage spatial de
		  5 coefficients operant sur les tableaux de reels simple
		  precision monodimensionnels.

*/


#include	"fafirf3.h"


/*
 * PROCEDURE	: firf3_float
 *
 * INPUT	:
 * src		Tableau de reels simple precision source.
 *
 * OUTPUT	:
 * dst		Tableau de reels simple precision destination.
 *
 * INPUT	:
 * n		Nombre d'elements des tableaux "a" et "b".
 * fir		Tableau des coefficients du filtre.
 *
 * DESCRIPTION	:
 * La procedure filtre le tableau de "n" reels simple precision "src" par le
 * filtre "fir" de 3 coefficients et stocke les elements filtres dans le
 * tableau de reels simple precision "dst".
 *
 * HISTORIQUE	:
 * 1.00 - 26/07/95 - Original.
 */
void	firf3_float (const float *src, float *dst, size_t n, const float *fir)
{
	const float	*send = src + n;	/* borne de "src"	*/

	for  (; src < send; src++, dst++)
		*dst = (src[0] * fir[0])
		     + (src[1] * fir[1])
		     + (src[2] * fir[2]);
}

/*
 * PROCEDURE	: firf3inc_float
 *
 * INPUT	:
 * src		Tableau de reels simple precision source.
 *
 * OUTPUT	:
 * dst		Tableau de reels simple precision destination.
 *
 * INPUT	:
 * n		Nombre d'elements a filtrer.
 * incsrc	Increment du tableau "src".
 * incdst	Increment du tableau "dst".
 * fir		Tableau des coefficients du filtre.
 *
 * DESCRIPTION	:
 * La procedure filtre "n" reels simple precision du tableau "src" par le
 * filtre "fir" de 3 coefficients et stocke les elements filtres dans le
 * tableau de reels simple precision "dst". Le parcourt des "n" reels source
 * s'effectue a partir du pointeur "src" par increment de "incsrc".
 * Le parcourt des "n" reels destination s'effectue a partir du pointeur
 * "dst" par increment de "incdst".
 *
 * NOTE		:
 * La procedure ne gere pas le cas du recouvrement en memoire des regions
 * pointees par "src" et "dst".
 *
 * HISTORIQUE	:
 * 1.00 - 27/07/95 - Original.
 */
void	firf3inc_float (const float *src, float *dst, size_t n,
		int incsrc, int incdst, const float *fir)
{
	size_t	inc1 = incsrc * 1;	/* multiples de l'increment	*/
	size_t	inc2 = incsrc * 2;

	size_t	i;	/* compteur	*/

	for (i = 0; i < n; i++) {
		*dst = (src[0]	  * fir[0])
		     + (src[inc1] * fir[1])
		     + (src[inc2] * fir[2]);
		src += incsrc;
		dst += incdst;
	}
}

/*
 * PROCEDURE	: firf3sym_float
 *
 * INPUT	:
 * src		Tableau de reels simple precision source.
 *
 * OUTPUT	:
 * dst		Tableau de reels simple precision destination.
 *
 * INPUT	:
 * n		Nombre d'elements des tableaux "a" et "b".
 * fir		Tableau des coefficients du filtre.
 *
 * DESCRIPTION	:
 * La procedure filtre le tableau de "n" reels simple precision "src" par le
 * filtre symetrique "fir" de 3 coefficients et stocke les elements filtres
 * dans le tableau de reels simple precision "dst".
 *
 * HISTORIQUE	:
 * 1.00 - 27/07/95 - Original.
 */
void	firf3sym_float (const float *src, float *dst, size_t n,
		const float *fir)
{
	const float	*send = src + n;	/* borne de "src"	*/

	for  (; src < send; src++, dst++)
		*dst = ((src[0] + src[2]) * fir[0])
		     + ((src[1])	  * fir[1]);
}

/*
 * PROCEDURE	: firf3syminc_float
 *
 * INPUT	:
 * src		Tableau de reels simple precision source.
 *
 * OUTPUT	:
 * dst		Tableau de reels simple precision destination.
 *
 * INPUT	:
 * n		Nombre d'elements des tableaux "a" et "b".
 * incsrc	Increment du tableau "src".
 * incdst	Increment du tableau "dst".
 * fir		Tableau des coefficients du filtre.
 *
 * DESCRIPTION	:
 * La procedure filtre le tableau de "n" reels simple precision "src" par le
 * filtre symetrique "fir" de 3 coefficients et stocke les elements filtres
 * dans le tableau de reels simple precision "dst". La procedure balaye le
 * tableau "src" par increment de "incsrc" et le tableau "dst" par increment
 * de "incdst".
 *
 * HISTORIQUE	:
 * 1.00 - 27/07/95 - Original.
 */
void	firf3syminc_float (const float *src, float *dst, size_t n,
		int incsrc, int incdst, const float *fir)
{
	size_t	inc1 = incsrc * 1;	/* multiples de l'increment	*/
	size_t	inc2 = incsrc * 2;

	size_t	i;	/* indice de pixel	*/

	for (i = 0; i < n; i++) {
		*dst = ((src[0]	   + src[inc2])	* fir[0])
		     + ((src[inc1])		* fir[1]);
		src += incsrc;
		dst += incdst;
	}
}


#ifdef	FAFIRF3_MAIN

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


#define	BUFSIZE		1000	/* taille des tableaux	*/
#define	FILTERSIZE	3

static	float	dst[BUFSIZE];
static	float	src[BUFSIZE];
static	float	fir[FILTERSIZE];


static	char	prog_name[] = "fafirf3";


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
 * 1.00 - 26/07/95 - Original.
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
 * 1.00 - 26/07/95 - Original.
 */
void	main (int argc, char *argv[])
{
	extern	long	atol (/* const char * */);

	float	*_src = &src[BUFSIZE - 1];
	float	*_dst = &dst[BUFSIZE - 1];
	long	basen;	/* base d'iterations	*/

	size_t	i;

	if (argc != 2) {
		print_usage ();
		exit (1);
	}

	basen = atol (argv[1]);

	for (i = 0; i < FILTERSIZE; i++)
		fir[i] = (float) rand ();

	LOOP(firf3_float (src, dst, BUFSIZE-3, fir), basen);
	LOOP(firf3inc_float (src, dst, BUFSIZE-3, 1, 1, fir), basen);
	LOOP(firf3sym_float (src, dst, BUFSIZE-3, fir), basen);
	LOOP(firf3syminc_float (_src, dst, BUFSIZE-3, -1, 1, fir), basen);

	exit (0);
}

#endif	/* FAFIRF3_MAIN	*/

