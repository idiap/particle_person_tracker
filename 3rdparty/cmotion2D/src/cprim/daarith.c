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
  
 DESCRIPTION	: Le fichier (daarith = Double Array ARITHmetic) contient les
		  procedures arithmetiques operant sur les tableaux de reels
		  double precision monodimensionnel.

*/


#include	<math.h>

#include	"daarith.h"


/*
 * PROCEDURE	: dot_double
 *
 * INPUT	:
 * a		Tableau de reels double precision operande droit.
 * b		Tableau de reels double precision operande droit.
 * n		Nombre d'elements dans les tableaux "a" et "b".
 *
 * DESCRIPTION	:
 * La procedure calcule le produit scalaire des tableaux "a" et "b" sur "n"
 * reels double precision : S a[i] x b[i].
 *
 * HISTORIQUE	:
 * 1.00 - 25/07/95 - Original.
 */
double	dot_double (const double *a, const double *b, size_t n)
{
#define	LSIZE	4

	const double	*aend = a + n;		/* borne de "a"		*/
	register double	sum   = (double) 0.0;	/* produit scalaire	*/

	if (n >= LSIZE) {	/* boucle deroulee LSIZE fois	*/
		aend -= LSIZE - 1;
		for (; a < aend; a += LSIZE, b += LSIZE) {
			sum += a[0] * b[0];
			sum += a[1] * b[1];
			sum += a[2] * b[2];
			sum += a[3] * b[3];
		}
		aend += LSIZE - 1;
	}
	for (; a < aend; sum += *a++ * *b++);
	return (sum);

#undef	LSIZE
}


#ifdef	DAARITH_MAIN

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
	((double) (_loopend-_loopstart) * 1e6) / (double) (CLICKSPERSEC * _n));\
}


#define	BUFSIZE	1000	/* taille des tableaux	*/


static	double	a[BUFSIZE];
static	double	b[BUFSIZE];

static	char	prog_name[] = "daarith";


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

	/* initailise les tableaux	*/
	for (i = 0; i < BUFSIZE; i++)
		a[i] = (double) rand ();
	for (i = 0; i < BUFSIZE; i++)
		b[i] = (double) rand ();


	LOOP(dot_double (a, b, BUFSIZE), basen);

	exit (0);
}

#endif	/* DAARITH_MAIN	*/

