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
  
 DESCRIPTION	: Le fichier (fafirf = Float Array Finite Impulse Response
		  Filter) contient les procedures de filtrage spatial operant
		  sur les tableaux de reels simple precision monodimensionnels.

*/


#include	"fafirf.h"


/*
 * PROCEDURE	: firf_float
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
 * fsize	Taille du filtre.
 *
 * DESCRIPTION	:
 * La procedure filtre le tableau de "n" reels simple precision "src" par le
 * filtre "fir" de "fsize"  coefficients et stocke les elements filtres dans le
 * tableau de reels simple precision "dst".
 *
 * HISTORIQUE	:
 * 1.00 - 26/07/95 - Original.
 */
void	firf_float (const float *src, float *dst, size_t n,
		const float *fir, size_t fsize)
{
	const float	*send = src + n;	/* borne de "src"	*/

	for  (; src < send; src++, dst++) {
		register float	sum = (float) 0.0;
		register size_t	i;

		for (i = 0; i < fsize; i++)
			sum += src[i] * fir[i];
		*dst = sum;
	}
}

/*
 * PROCEDURE	: firfsym_float
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
 * fsize	Taille du filtre.
 *
 * DESCRIPTION	:
 * La procedure filtre le tableau de "n" reels simple precision "src" par le
 * filtre symetrique "fir" de "fsize" coefficients et stocke les elements
 * filtres dans le tableau de reels simple precision "dst".
 *
 * HISTORIQUE	:
 * 1.00 - 27/07/95 - Original.
 */
void	firfsym_float (const float *src, float *dst, size_t n,
		const float *fir, size_t fsize)
{
	const float	*send = src + n;	/* borne de "src"	*/
	size_t		 hsize = fsize / 2;	/* demi-taille filtre	*/

	for  (; src < send; src++, dst++) {
		register float	sum;
		register size_t	i;

		sum = src[hsize] * fir[hsize];
		for (i = 0; i < hsize; i++)
			sum += (src[i] + src[fsize - 1 - i]) * fir[i];
		*dst = sum;
	}
}

/*
 * PROCEDURE	: firfsyminc_float
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
 * fsize	Taille du filtre.
 *
 * DESCRIPTION	:
 * La procedure filtre le tableau de "n" reels simple precision "src" par le
 * filtre symetrique "fir" de "fsize" coefficients et stocke les elements
 * filtres dans le tableau de reels simple precision "dst". La procedure balaye
 * le tableau "src" par increment de "incsrc" et le tableau "dst" par increment
 * de "incdst".
 *
 * HISTORIQUE	:
 * 1.00 - 27/07/95 - Original.
 */
void	firfsyminc_float (const float *src, float *dst, size_t n,
		int incsrc, int incdst, const float *fir, size_t fsize)
{
	size_t	hsize = fsize / 2;	/* demi-taille filtre	*/
	size_t	i;			/* compteur		*/

	if (incsrc < 0) src += (n - 1) * (size_t) -incsrc;
	if (incdst < 0) dst += (n - 1) * (size_t) -incdst;

	for (i = 0; i < n; i++) {
		register float	sum;
		register size_t	j;

		sum = src[hsize] * fir[hsize];
		for (j = 0; j < hsize; j++)
			sum += (src[j] + src[fsize - 1 - j]) * fir[j];
		*dst = sum;

		src += incsrc;
		dst += incdst;
	}
}


#ifdef	FAFIRF_MAIN

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
#define	FILTERSIZE	5

static	float	dst[BUFSIZE];
static	float	src[BUFSIZE];
static	float	fir[FILTERSIZE];


static	char	prog_name[] = "fafirf";


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
void	main (argc, argv, envp)
int	argc;
char	*argv[], *envp[];
{
	extern	long	atol (/* const char * */);

	long	basen;	/* base d'iterations	*/
	size_t	i;

	if (argc != 2) {
		print_usage ();
		exit (1);
	}

	basen = atol (argv[1]);

	for (i = 0; i < FILTERSIZE; i++)
		fir[i] = (float) rand ();

	LOOP(firf_float (src, dst, BUFSIZE-1-5, fir, 5), basen);
	LOOP(firfsym_float (src, dst, BUFSIZE-1-5, fir, 5), basen);
	LOOP(firfsyminc_float (src, dst, BUFSIZE-1-5, -1, 1, fir, 5), basen);

	exit (0);
}

#endif	/* FAFIRF_MAIN	*/

