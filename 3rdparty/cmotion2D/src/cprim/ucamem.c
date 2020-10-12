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
  
 DESCRIPTION	: Le fichier (ucamem = Unsigned Char Array MEMory) contient les
		  procedures de gestion en memoire des tableaux de caracteres
		  non signes monodimensionnels.

*/


#include	<memory.h>

#include	"ucamem.h"


typedef	unsigned char	uchar;	/* POSIX ?	*/


/*
 * PROCEDURE	: copy_uchar
 *
 * INPUT	:
 * src		Tableau de caracteres non signes source.
 *
 * OUTPUT	:
 * dst		Tableau de caracteres non signes destination.
 *
 * INPUT	:
 * n		Nombre d'elements du tableau "src" a copier.
 *
 * DESCRIPTION	:
 * La procedure copie "n" caracteres non signes du tableau "src" dans le
 * tableau "dst".
 *
 * HISTORIQUE	:
 * 1.00 - 18/07/95 - Original.
 */
void	copy_uchar (const unsigned char *src, unsigned char *dst, size_t n)
{
	(void) memcpy ((void *) dst, (void *) src, n * sizeof(uchar));
}


/*
 * PROCEDURE	: copyinc_uchar
 *
 * INPUT	:
 * src		Tableau de caracteres non signes source.
 *
 * OUTPUT	:
 * dst		Tableau de caracteres non signes destination.
 *
 * INPUT	:
 * n		Nombre d'elements du tableau "src" a copier.
 * incsrc	Increment du tableau "src".
 * incdst	Increment du tableau "dst".
 *
 * DESCRIPTION	:
 * La procedure copie "n" caracteres non signes du tableau "src" dans le
 * tableau "dst". Le parcourt des "n" caracteres non signes source s'effectue
 * a partir du pointeur "src" par increment de "incsrc". Le parcourt des "n"
 * caracteres non signes destination s'effectue a partir du pointeur "dst" par
 * increment de "incdst".
 *
 * HISTORIQUE	:
 * 1.00 - 13/03/97 - Original.
 */
void	copyinc_uchar (const unsigned char *src, unsigned char *dst, size_t n,
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
 * PROCEDURE	: set_uchar
 *
 * OUTPUT	:
 * buf		Tableau de caracteres non signes a initialiser.
 *
 * INPUT	:
 * v		Valeur d'initialisation du tableau.
 * n		Nombre d'elements du tableau "buf" a initialiser.
 *
 * DESCRIPTION	:
 * La procedure initialise "n" caracteres non signes du tableau "buf" par la
 * valeur "v".
 *
 * HISTORIQUE	:
 * 1.00 - 18/07/95 - Original.
 */
void	set_uchar (unsigned char *buf, unsigned char v, size_t n)
{
	(void) memset ((void *) buf, (int) v, n * sizeof(uchar));
}


#ifdef	UCAMEM_MAIN

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


static	uchar	buf[BUFSIZE];
static	uchar	dst[BUFSIZE];
static	uchar	src[BUFSIZE];

static	char	prog_name[] = "ucamem";


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
 * 1.00 - 18/07/95 - Original.
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
 * 1.00 - 18/07/95 - Original.
 */
int	main (int argc, char *argv[])
{
	long	basen;	/* base d'iterations	*/

	if (argc != 2) {
		print_usage ();
		exit (1);
	}

	basen = atol (argv[1]);

	LOOP((void) copy_uchar (src, dst, BUFSIZE), basen);
	LOOP((void) set_uchar (buf, 0, BUFSIZE), basen);
	LOOP(copyinc_uchar (src, dst, BUFSIZE, 1, 1), basen);

	return (0);
}

#endif	/* UCAMEM_MAIN	*/

