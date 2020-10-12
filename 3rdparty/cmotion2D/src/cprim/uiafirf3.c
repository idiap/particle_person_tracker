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

  DESCRIPTION	: Le fichier (uiafirf3 = Unsigned Int Array Finite Impulse Response
		  Filter 3) contient les procedures de filtrage spatial de
		  3 coefficients operant sur les tableaux d'entiers non signes
		  monodimensionnels.

*/


#include	"uiafirf3.h"


/*
 * PROCEDURE	: firf3_uint
 *
 * INPUT	:
 * src		Tableau d'entiers non signe source.
 *
 * OUTPUT	:
 * dst		Tableau d'entiers non signe destination.
 *
 * INPUT	:
 * n		Nombre d'elements des tableaux "a" et "b".
 * fir		Tableau des coefficients du filtre.
 *
 * DESCRIPTION	:
 * La procedure filtre le tableau de "n" entiers non signes "src" par le
 * filtre "fir" de 3 coefficients et stocke les elements filtres dans le
 * tableau d'entiers non signes "dst".
 *
 * HISTORIQUE	:
 * 1.00 - 26/07/95 - Original.
 */
void	firf3_uint (const unsigned int *src, unsigned int *dst, size_t n)
{
	const unsigned int	*send = src + n;	/* borne de "src"	*/

	for  (; src < send; src++, dst++)
		*dst = (src[0] + src[1] + src[2]);
}



#ifdef	UIAFIRF3_MAIN

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

static	uint	dst[BUFSIZE];
static	uint	src[BUFSIZE];


static	char	prog_name[] = "uiafirf3";


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
int	main (int argc, char *argv[])
{

	uint	*_src = &src[BUFSIZE - 1];
	uint	*_dst = &dst[BUFSIZE - 1];
	long	basen;	/* base d'iterations	*/

	if (argc != 2) {
		print_usage ();
		exit (1);
	}

	basen = atol (argv[1]);

	LOOP(firf3_uint (src, dst, BUFSIZE-3), basen);

	exit (0);
}

#endif	/* UIAFIRF3_MAIN	*/

