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
  
 DESCRIPTION	: Le fichier (ucaarith = Unsigned Char integer Array ARITHmetic)
		  contient les procedures arithmetiques operant sur les
		  tableaux d'entiers courts monodimensionnels.

*/


#include	<errno.h>


#include	"ucaarith.h"




/*
 * PROCEDURE	: add_uchar
 *
 * OUTPUT	:
 * a		Tableau de caracteres operande gauche et droit.
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
void	add_uchar (uchar *a, const uchar *b, size_t n)
{
#define	LSIZE	4

	uchar	*aend = a + n;	/* borne de "a"	*/

	if (n >= LSIZE) {	/* boucle deroulee LSIZE fois	*/
		aend -= LSIZE - 1;
		for (; a < aend; a += LSIZE, b += LSIZE) {
			a[0] = (unsigned char) (a[0] + b[0]);
			a[1] = (unsigned char) (a[1] + b[1]);
			a[2] = (unsigned char) (a[2] + b[2]);
			a[3] = (unsigned char) (a[3] + b[3]);
		}
		aend += LSIZE - 1;
	}
	for (; a < aend; *a++ += *b++);

#undef	LSIZE
}


/*
 * PROCEDURE	: subtract_uchar
 *
 * OUTPUT	:
 * a		Tableau de caracteres operande gauche et droit.
 *
 * INPUT	:
 * b		Tableau de caracteres courts operande droit.
 * n		Nombre d'elements dans les tableaux "a" et "b".
 *
 * DESCRIPTION	:
 * La procedure soustrait du tableau de caractetes "a" le tableau de
 * caracteres "b" : a[i] -= b[i].
 *
 * HISTORIQUE	:
 * 1.00 - 27/06/90 - Original.
 */
void	subtract_uchar (uchar *a, const uchar *b, size_t n)
{
#define	LSIZE	4

	uchar	*aend = a + n;	/* borne de "a"	*/

	if (n >= LSIZE) {	/* boucle deroulee LSIZE fois	*/
		aend -= LSIZE - 1;
		for (; a < aend; a += LSIZE, b += LSIZE) {
			a[0] = (unsigned char) (a[0] - b[0]);
			a[1] = (unsigned char) (a[1] - b[1]);
			a[2] = (unsigned char) (a[2] - b[2]);
			a[3] = (unsigned char) (a[3] - b[3]);
		}
		aend += LSIZE - 1;
	}
	for (; a < aend; *a++ -= *b++);

#undef	LSIZE
}


#ifdef	UCAARITH_MAIN

#define	BASEN		1000000

#include	<stdio.h>
#include	<stdlib.h>
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


static	uchar	a[BUFSIZE];
static	uchar	b[BUFSIZE];


static	char	prog_name[] = "ucaarith";


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
int	main (int argc, char *argv[])
{
	long	basen;	/* base d'iterations	*/
	uchar	low  = BUFSIZE / 10;
	uchar	high = (BUFSIZE * 9) / 10;
	uchar   in   = 0;
	uchar	out  = BUFSIZE - 1;
	size_t	i;

	if (argc != 2) {
		print_usage ();
		exit (1);
	}

	basen = atol (argv[1]);

	/* initailise les tableaux	*/
	for (i = 0; i < BUFSIZE; i++)
		a[i] = (uchar) (rand () % BUFSIZE);
	for (i = 0; i < BUFSIZE; i++)
		b[i] = (uchar) (rand () % BUFSIZE);


	LOOP(abs_uchar (a, BUFSIZE), basen);
	LOOP(add_uchar (a, b, BUFSIZE), basen);
	LOOP(subtract_uchar (a, b, BUFSIZE), basen);

	exit (0);
}

#endif	/* UCAARITH_MAIN	*/

