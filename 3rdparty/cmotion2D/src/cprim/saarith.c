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
  
 DESCRIPTION	: Le fichier (saarith = Short integer Array ARITHmetic)
		  contient les procedures arithmetiques operant sur les
		  tableaux d'entiers courts monodimensionnels.

*/


#include	<errno.h>

#include	"saarith.h"


/*
 * PROCEDURE	: abs_short
 *
 * OUTPUT	:
 * a		Tableau d'entiers courts operande gauche et droit.
 *
 * INPUT	:
 * n		Nombre d'elements dans le tableau "a".
 *
 * DESCRIPTION	:
 * La procedure calcule la valeur absolue du tableau d'entiers courts "a" :
 * a[i] = abs(a[i]).
 *
 * HISTORIQUE	:
 * 1.00 - 27/06/90 - Original.
 */
void	abs_short (short *a, size_t n)
{
#define	LSIZE	4

	short	*aend = a + n;	/* borne de "a"	*/

	if (n >= LSIZE) {	/* boucle deroulee LSIZE fois	*/
		aend -= LSIZE - 1;
		for (; a < aend; a += LSIZE) {
			if (a[0] < 0) a[0] = (short) -a[0];
			if (a[1] < 0) a[1] = (short) -a[1];
			if (a[2] < 0) a[2] = (short) -a[2];
			if (a[3] < 0) a[3] = (short) -a[3];
		}
		aend += LSIZE - 1;
	}
	for (; a < aend; a++)
		if (*a < 0) *a = (short) -*a;

#undef	LSIZE
}

/*
 * PROCEDURE	: add_short
 *
 * OUTPUT	:
 * a		Tableau d'entiers courts operande gauche et droit.
 *
 * INPUT	:
 * b		Tableau d'entiers courts operande droit.
 * n		Nombre d'elements dans les tableaux "a" et "b".
 *
 * DESCRIPTION	:
 * La procedure additionne au tableau d'entiers courts "a" le tableau d'entiers
 * courts "b" : a[i] += b[i].
 *
 * HISTORIQUE	:
 * 1.00 - 27/06/90 - Original.
 */
void	add_short (short *a, const short *b, size_t n)
{
#define	LSIZE	4

	short	*aend = a + n;	/* borne de "a"	*/

	if (n >= LSIZE) {	/* boucle deroulee LSIZE fois	*/
		aend -= LSIZE - 1;
		for (; a < aend; a += LSIZE, b += LSIZE) {
			a[0] = (short) (a[0] + b[0]);
			a[1] = (short) (a[1] + b[1]);
			a[2] = (short) (a[2] + b[2]);
			a[3] = (short) (a[3] + b[3]);
		}
		aend += LSIZE - 1;
	}
	for (; a < aend; *a++ += *b++);

#undef	LSIZE
}

/*
 * PROCEDURE	: binary_short
 *
 * OUTPUT	:
 * a		Tableau d'entiers courts operande gauche et droit.
 *
 * INPUT	:
 * n		Nombre d'elements dans le tableau "a".
 * low		Seuil bas de binarisation.
 * high		Seuil haut de binarisation.
 * in		Valeur dans l'intervalle.
 * out		Valeur hors l'intervalle.
 *
 * DESCRIPTION	:
 * La procedure binarise les "n" entiers courts du tableau "a" :
 * a[i] = in	si low <= a[i] <= high,
 * a[i] = out	sinon.
 *
 * HISTORIQUE	:
 * 1.00 - 16/06/95 - Original.
 */
void	binary_short (short *a, size_t n, short low, short high,
		short in, short out)
{
#define	LSIZE	4

	register short	*aend = a + n;	/* borne de "a"	*/

	if (n >= LSIZE) {	/* boucle deroulee LSIZE fois	*/
		aend -= LSIZE - 1;
		for (; a < aend; a += LSIZE) {
			a[0] = (low <= a[0] && a[0] <= high) ? in : out;
			a[1] = (low <= a[1] && a[1] <= high) ? in : out;
			a[2] = (low <= a[2] && a[2] <= high) ? in : out;
			a[3] = (low <= a[3] && a[3] <= high) ? in : out;
		}
		aend += LSIZE - 1;
	}
	for (; a < aend; a++)
		*a = (low <= *a && *a <= high) ? in : out;

#undef	LSIZE
}

/*
 * PROCEDURE	: divconst_short
 *
 * OUTPUT	:
 * a		Tableau d'entiers courts operande gauche et droit.
 *
 * INPUT	:
 * d		Valeur de division operande droit.
 * n		Nombre d'elements dans le tableau "a".
 *
 * DESCRIPTION	:
 * La procedure divise le tableau d'entiers courts "a" par la valeur "d" :
 * a[i] /= d.
 *
 * HISTORIQUE	:
 * 1.00 - 27/06/90 - Original.
 */
void	divconst_short (short * a, short d, size_t n)
{
#define	LSIZE	4

	short		*aend = a + n;	/* borne de "a"	*/
	register short	t = d;		/* optimise ?	*/

	if (t == 0)
		return;

	if (n >= LSIZE) {	/* boucle deroulee LSIZE fois	*/
		aend -= LSIZE - 1;
		for (; a < aend; a += LSIZE) {
			a[0] = (short) (a[0] / t);
			a[1] = (short) (a[1] / t);
			a[2] = (short) (a[2] / t);
			a[3] = (short) (a[3] / t);
		}
		aend += LSIZE - 1;
	}
	for (; a < aend; *a++ /= t);

#undef	LSIZE
}

/*
 * PROCEDURE	: histogram_short
 *
 * INPUT	:
 * a		Tableau d'entiers courts en entree.
 * n		Nombre d'elements dans le tableau "a".
 *
 * OUTPUT	:
 * lut		Tableau cumulant l'histogramme du tableau.
 *
 * DESCRIPTION	:
 * La procedure cumule l'histogramme du tableau d'entiers courts "a"
 * a l'histogramme "lut" : lut += histogramme (a).
 *
 * NOTE		:
 * L'histogramme n'est pas mis a zero avant passage sur le tableau.
 * La procedure ne controle pas la dynamique du tableau vis-a-vis du nombre
 * d'entrees de l'histogramme.
 *
 * HISTORIQUE	:
 * 1.00 - 27/06/90 - Original.
 */
void	histogram_short (const short *a, size_t n, long *lut)
{
#define	LSIZE	4

	const short	*aend = a + n;	/* borne de "a"	*/

	if (n >= LSIZE) {	/* boucle deroulee LSIZE fois	*/
		aend -= LSIZE - 1;
		for (; a < aend; a += LSIZE) {
			lut[a[0]]++;
			lut[a[1]]++;
			lut[a[2]]++;
			lut[a[3]]++;
		}
		aend += LSIZE - 1;
	}
	for (; a < aend; lut[*a++]++);

#undef	LSIZE
}

/*
 * PROCEDURE	: lut_short
 *
 * OUTPUT	:
 * a		Tableau d'entiers courts operande gauche et droit.
 *
 * INPUT	:
 * n		Nombre d'elements dans le tableau "a".
 * lut		Table de transcodage (Look-Up Table).
 *
 * DESCRIPTION	:
 * La procedure transcode le tableau d'entiers courts "a" par la table de
 * transcodage "lut" : a[i] = lut[a[i]].
 *
 * HISTORIQUE	:
 * 1.00 - 27/06/90 - Original.
 */
void	lut_short (short *a, size_t n, const short *lut)
{
#define	LSIZE	4

	short	*aend = a + n;	/* borne de "a"	*/

	if (n >= LSIZE) {	/* boucle deroulee LSIZE fois	*/
		aend -= LSIZE - 1;
		for (; a < aend; a += LSIZE) {
			a[0] = lut[a[0]];
			a[1] = lut[a[1]];
			a[2] = lut[a[2]];
			a[3] = lut[a[3]];
		}
		aend += LSIZE - 1;
	}
	for (; a < aend; a++)
		*a = lut[*a];

#undef	LSIZE
}

/*
 * PROCEDURE	: minmax_short
 *
 * GLOBAL	:
 * errno	Numero de l'erreur systeme courante (voir perror(3)).
 *
 * INPUT	:
 * a		Tableau d'entiers courts en entree.
 * n		Nombre d'elements dans le tableau "a".
 *
 * OUTPUT	:
 * minp		Indice de la valeur minimale du tableau.
 * maxp		Indice de la valeur maximale du tableau.
 *
 * DESCRIPTION	:
 * La procedure recherche dans le tableau d'entiers courts "a" de taille "n"
 * les indices de la valeur minimale "minp" et de la valeur maximale "maxp".
 *
 * RETOUR	:
 * En cas de succes, une valeur non negative est retournee.
 * Sinon, la valeur -1 est retournee et la variable globale "errno" est
 * initialisee pour indiquer la cause de l'erreur :
 * EINVAL	Le nombre d'elements dans le tableau est nul, la recherche
 *		est impossible.
 *
 * HISTORIQUE	:
 * 1.00 - 27/06/90 - Original.
 * 1.01 - 05/07/95 - Retypage de "n" de "int" en "size_t".
 */
int	minmax_short (const short *a, size_t n, size_t *minp, size_t *maxp)
{
	register short	vmin, vmax;	/* valeurs des bornes	*/
	register size_t	imin, imax;	/* indices des bornes	*/
	register size_t	i;

	if (n < 1) { 	/* tableau a vide	*/
		errno = EINVAL;
		return (-1);
	}

	vmax = vmin = a[imax = imin = 0];
	for (i = 1; i < n; i++) {
		if (a[i] < vmin)
			vmin = a[imin = i];
		else if (a[i] > vmax)
			vmax = a[imax = i];
	}
	*minp = imin;
	*maxp = imax;
	return (0);
}

/*
 * PROCEDURE	: multiply_short
 *
 * OUTPUT	:
 * a		Tableau d'entiers courts operande gauche et droit.
 *
 * INPUT	:
 * b		Tableau d'entiers courts operande droit.
 * n		Nombre d'elements dans les tableaux "a" et "b".
 *
 * DESCRIPTION	:
 * La procedure multiplie le tableaux d'entiers courts "a" par le tableau
 * d'entiers courts "b" : a[i] *= b[i].
 *
 * HISTORIQUE	:
 * 1.00 - 27/06/90 - Original.
 */
void	multiply_short (short *a, const short *b, size_t n)
{
#define	LSIZE	4

	short	*aend = a + n;	/* borne de "a"	*/

	if (n >= LSIZE) {	/* boucle deroulee LSIZE fois	*/
		aend -= LSIZE - 1;
		for (; a < aend; a += LSIZE, b += LSIZE) {
			a[0] = (short) (a[0] * b[0]);
			a[1] = (short) (a[1] * b[1]);
			a[2] = (short) (a[2] * b[2]);
			a[3] = (short) (a[3] * b[3]);
		}
		aend += LSIZE - 1;
	}
	for (; a < aend; *a++ *= *b++);

#undef	LSIZE
}

/*
 * PROCEDURE	: negpos_short
 *
 * INPUT	:
 * a		Tableau d'entiers courts a parcourir.
 * n		Nombre d'elements dans le tableau "a".
 *
 * OUTPUT	:
 * negp		Pointeur sur la somme des elements negatifs de "a".
 * posp		Pointeur sur la somme des elements positifs de "a".
 *
 * DESCRIPTION	:
 * La procedure somme a partir du tableau d'entiers courts "a" les elements
 * negatifs dans "negp" et les elements positifs dans "posp".
 *
 * HISTORIQUE	:
 * 1.00 - 26/12/91 - Original.
 */
void	negpos_short (const short *a, size_t n, long *negp, long *posp)
{
	long		sumneg = 0L;	/* somme des elements negatifs	*/
	long		sumpos = 0L;	/* somme des elements positifs	*/
	register size_t	i;

	for (i = 0; i < n; i++) {
		if (a[i] < 0)	sumneg += (long) a[i];
		else 		sumpos += (long) a[i];
	}
	*negp = sumneg;
	*posp = sumpos;
}

/*
 * PROCEDURE	: subtract_short
 *
 * OUTPUT	:
 * a		Tableau d'entiers courts operande gauche et droit.
 *
 * INPUT	:
 * b		Tableau d'entiers courts operande droit.
 * n		Nombre d'elements dans les tableaux "a" et "b".
 *
 * DESCRIPTION	:
 * La procedure soustrait du tableau d'entiers courts "a" le tableau d'entiers
 * courts "b" : a[i] -= b[i].
 *
 * HISTORIQUE	:
 * 1.00 - 27/06/90 - Original.
 */
void	subtract_short (short *a, const short *b, size_t n)
{
#define	LSIZE	4

	short	*aend = a + n;	/* borne de "a"	*/

	if (n >= LSIZE) {	/* boucle deroulee LSIZE fois	*/
		aend -= LSIZE - 1;
		for (; a < aend; a += LSIZE, b += LSIZE) {
			a[0] = (short) (a[0] - b[0]);
			a[1] = (short) (a[1] - b[1]);
			a[2] = (short) (a[2] - b[2]);
			a[3] = (short) (a[3] - b[3]);
		}
		aend += LSIZE - 1;
	}
	for (; a < aend; *a++ -= *b++);

#undef	LSIZE
}


#ifdef	SAARITH_MAIN

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


static	short	a[BUFSIZE];
static	short	b[BUFSIZE];
static	long	hist[BUFSIZE];
static	short	lut[BUFSIZE];


static	char	prog_name[] = "saarith";


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
	long	sumneg, sumpos;
	short	low  = BUFSIZE / 10;
	short	high = (BUFSIZE * 9) / 10;
	short   in   = 0;
	short	out  = BUFSIZE - 1;
	size_t	imin, imax;
	size_t	i;

	if (argc != 2) {
		print_usage ();
		exit (1);
	}

	basen = atol (argv[1]);

	/* initailise les tableaux	*/
	for (i = 0; i < BUFSIZE; i++)
		a[i] = (short) (rand () % BUFSIZE);
	for (i = 0; i < BUFSIZE; i++)
		b[i] = (short) (rand () % BUFSIZE);


#if	0
	LOOP(abs_short (a, BUFSIZE), basen);
#endif
	LOOP(add_short (a, b, BUFSIZE), basen);
	LOOP(multiply_short (a, b, BUFSIZE), basen);
#if	0
	LOOP(binary_short (a, BUFSIZE, low, high, in, out), basen);
	LOOP(divconst_short (a, 1, BUFSIZE), basen);

	abs_short (a, BUFSIZE);
	for (i = 0; i < BUFSIZE; i++)
		a[i] %= BUFSIZE;
	for (i = 0; i < BUFSIZE; i++)
		lut[i] = BUFSIZE - i - 1;

	LOOP(histogram_short (a, BUFSIZE, hist), basen);
	LOOP(lut_short (a, BUFSIZE, lut), basen);

	LOOP(minmax_short (a, BUFSIZE, &imin, &imax), basen);
	LOOP(multiply_short (a, b, BUFSIZE), basen);
	LOOP(negpos_short (a, BUFSIZE, &sumneg, &sumpos), basen);
#endif
	LOOP(subtract_short (a, b, BUFSIZE), basen);

	exit (0);
}

#endif	/* SAARITH_MAIN	*/

