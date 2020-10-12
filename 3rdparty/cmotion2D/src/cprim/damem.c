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
  
 DESCRIPTION	: Le fichier (damem = Double Array MEMory) contient les
		  procedures de gestion en memoire des tableaux de reels
		  double precision monodimensionnels.

*/


#include	<memory.h>

#include	"damem.h"


/*
 * PROCEDURE	: copy_double
 *
 * INPUT	:
 * src		Tableau de reels double precision source.
 *
 * OUTPUT	:
 * dst		Tableau de reels double precision destination.
 *
 * INPUT	:
 * n		Nombre d'elements du tableau "src" a copier.
 *
 * DESCRIPTION	:
 * La procedure copie "n" reels double precision du tableau "src" dans le
 * tableau "dst".
 *
 * HISTORIQUE	:
 * 1.00 - 17/12/91 - Original.
 */
void	copy_double (register double *src, register double *dst, size_t	n)
{
#define	LSIZE	4

	register double	*send = src + n;	/* borne de "src"	*/

	if (n >= LSIZE) {	/* boucle deroulee LSIZE fois	*/
		send -= LSIZE - 1;
		for (; src < send; src += LSIZE, dst += LSIZE) {
			dst[0] = src[0];
			dst[1] = src[1];
			dst[2] = src[2];
			dst[3] = src[3];
		}
		send += LSIZE - 1;
	}
	for (; src < send; *dst++ = *src++);

#undef	LSIZE
	/*
 	 * (void) memcpy ((char *) dst, (char *) src, n * sizeof(double));
	 */
}

/*
 * PROCEDURE	: set_double
 *
 * OUTPUT	:
 * buf		Tableau de reels double precision a initialiser.
 *
 * INPUT	:
 * v		Valeur d'initialisation du tableau.
 * n		Nombre d'elements du tableau "buf" a initialiser.
 *
 * DESCRIPTION	:
 * La procedure initialise "n" reels double precision du tableau "buf" par la
 * valeur "v".
 *
 * HISTORIQUE	:
 * 1.00 - 17/12/91 - Original.
 */
void	set_double (register double *buf, double v, size_t n)
{
#define	LSIZE	4

	register double	*bend = buf + n;	/* borne de "buf"	*/
	register double	t = v;			/* optimise ?		*/

	if (n >= LSIZE) {	/* boucle deroulee LSIZE fois	*/
		bend -= LSIZE - 1;
		for (; buf < bend; buf += LSIZE) {
			buf[0] = t;
			buf[1] = t;
			buf[2] = t;
			buf[3] = t;
		}
		bend += LSIZE - 1;
	}
	for (; buf < bend; *buf++ = t);

#undef	LSIZE
}


#ifdef	DAMEM_MAIN

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
#define	quoted(TEXT)	"TEXT"

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


static	double	dst[BUFSIZE];
static	double	src[BUFSIZE];

static	char	prog_name[] = "damem";


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
void	main (int argc, char *argv[], char *envp[])
{
	extern	long	atol (/* const char * */);

	long	basen;	/* base d'iterations	*/

	if (argc != 2) {
		print_usage ();
		exit (1);
	}
	basen = atol (argv[1]);

	LOOP(copy_double (src, dst, BUFSIZE), basen);
	LOOP(set_double (src, 0.0, BUFSIZE), basen);

	exit (0);
}

#endif	/* DAMEM_MAIN	*/

