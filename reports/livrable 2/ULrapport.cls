%!TEX encoding = UTF-8 Unicode

%
% Classe de document "ULrapport" 
% par Pierre Tremblay, Universite Laval
% modifiee par Christian Gagne, Universite Laval
% modifiee par Francis Valois
% Version 2.0, 3/02/2011
%

%
% Historique des versions
%
% 2011/02/03 - version 2.0
% Modification globale pour être acceptable en Électronique des composants discret
%
% 2011/01/14 - version 1.1h
% Correction du répertoire de figures: répertoire fig pour toutes figures.
% Ne pas spécifier les extensions, laisser LaTeX choisir le format approprié.
% Ajout d'information sur l'encodage des fichier pour TeXshop (Mac).
% Mise-à-jour du rapport exemple en conséquence.
% Formatage plus léger des commentaires de la classe et des exemples
%
% 2008/03/28 - version 1.1g
% Repositionnement de "\addcontentsline" pour assurer un hyperlien correct (3 fois)
% par Jean-Dominic Laroche (GLO)
% Ajout du package "url" avec option "hyphens" pour faciliter la cesure des urls
% Ajout de la commande "\raggedright" a la bibliographie pour faciliter la mise en page
% et eliminer les nombreux "underfull" autrement generes
%
% 2008/02/08 - version 1.1f
% Utilisation de \parbox pour presenter les titres longs avec un espacement correct
%
% 2007/12/02 - version 1.1e
% Ajout de la bibliographie et des listes a la table des matieres par l'environnement
% "thebibliographyUL"
% Ajout de l'historique des versions
%
% 2007/05/05 - version 1.1d
% Ajout des compteurs "tocdepth" et "secnumdepth" pour la numerotation des sections
% Ajout des flags logiques pour l'insertion facultative des listes (figures, tableaux)
%
% 2007/04/12 - version 1.1c
% Changement de "\theEquipe" pour "\theNumeroEquipe" et "\theNomEquipe"
% Ajout des flags logiques pour l'insertion facultative des listes (figures, tableaux)
%
% 2007/02/01 - version 1.1b
% Retrait de l'option d'hyperref "pdfpagelables=true", car deja par defaut
%
% 2007/01/17 - version 1.1a
% Ajout de "\renewcommand{\arraystretch}{1.5}" a la page titre
%
% 2007/01/16 - version 1.1 
% Mise a jour complete
%
% 2006/03/01 - version 1.0
% Creation
%

%--------------------------------------------------------------------------------------
%----------------------------------- identification -----------------------------------
%--------------------------------------------------------------------------------------
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{ULrapport}%[2011/01/14 v1.1h Classe de rapport technique - Universite Laval]
              
%--------------------------------------------------------------------------------------
%------------------------------------ initial code ------------------------------------
%--------------------------------------------------------------------------------------

% Flags logiques permettant l'insertion des listes des figures et des tableaux
\RequirePackage{ifthen}
\newboolean{InsererListeDesFigures}
\newboolean{InsererListeDesTableaux}


%--------------------------------------------------------------------------------------
%------------------------------- declaration of options -------------------------------
%--------------------------------------------------------------------------------------
\DeclareOption{ULlof}{\setboolean{InsererListeDesFigures}{true}}
\DeclareOption{ULlot}{\setboolean{InsererListeDesTableaux}{true}}
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{report}}

%--------------------------------------------------------------------------------------
%-------------------------------- execution of options --------------------------------
%--------------------------------------------------------------------------------------
\ProcessOptions \relax

%--------------------------------------------------------------------------------------
%----------------------------------- package loading ----------------------------------
%--------------------------------------------------------------------------------------
\LoadClass{report}
\RequirePackage[utf8]{inputenc}
\RequirePackage{geometry}
\RequirePackage{lmodern}
\RequirePackage[T1]{fontenc}
\RequirePackage[french]{babel}
\RequirePackage[font={it}]{caption}
\RequirePackage{textcomp}        % pour \textcelsius, \textmu, \textohm
\RequirePackage{array}
\RequirePackage{amstext}         % pour \text
\RequirePackage{tabularx}
\RequirePackage{colortbl}
\RequirePackage{longtable}
\RequirePackage{graphicx}
\RequirePackage{rotating}
% \RequirePackage{mathabx}         % pour \second (peut etre remplace par "\prime\prime")
% \RequirePackage{color}
\RequirePackage[hyphens]{url}
\RequirePackage{hyperref}
\RequirePackage{multirow}
\RequirePackage{amsmath}
\RequirePackage{SIunits}
\RequirePackage{subfigure}

%--------------------------------------------------------------------------------------
%-------------------------------------- main code -------------------------------------
%--------------------------------------------------------------------------------------

% Definitions des parametres du package hyperref
\hypersetup{plainpages=false,%
            breaklinks=false,%
            colorlinks=true,%
            menucolor=black,%
            linkcolor=black}

% Definitions des dimensions de la page (package geometry)
\geometry{letterpaper,%
          centering,%
          hmargin={2.5cm,2.5cm},%
          vmargin={2.5cm,2.5cm},%
          heightrounded,%
          includehead}

% Modification du Header des pages Chapitre
\def\@makechapterhead#1{%
  {\parindent \z@ \raggedright \normalfont
    \ifnum \c@secnumdepth >\m@ne
        \huge\bfseries \@chapapp\space \thechapter
        \par\nobreak
        \vskip 20\p@
    \fi
    \interlinepenalty\@M
    \Huge \bfseries #1\par\nobreak
    \vskip 40\p@
  }}
\makeatother

% Utilisation du package "caption" pour
%    - redefinit le caractere de separation dans les legendes comme "babel" le definit,
%      car "caption" (ou "float", le cas echeant) le redefinit comme ":"
%    - uniformise les legendes entre table, longtable et supertabular
%    - specifie que la legende des tableaux sera placee en haut,
%      donc intervertit les dimensions d'espacements (avant et apres la legende)
\DeclareCaptionLabelSeparator{as-Babel-french}{\space\textendash\space}
\captionsetup{margin=0.1\textwidth,labelsep=as-Babel-french}
\captionsetup[table]{position=top}

%Override le titre de tableau et de figure
\addto\captionsfrench{\def\tablename{Tableau}}
\addto\captionsfrench{\def\figurename{Figure}}

% Redefinition de l'environnement "thebibliography" afin que son existence soit
% mentionnee a la table des matieres
\newenvironment{thebibliographyUL}[1]%
               {\clearpage%
                \begin{thebibliography}{#1}%
                \addcontentsline{toc}{chapter}{\bibname}%
                \raggedright%
               }%
               {\end{thebibliography}}
               

% En-tete et bas de page par defaut
\pagestyle{headings}


% Definitions pour l'inclusion de graphiques
%    - des repertoires par defaut
%    - des extensions supportees
\graphicspath{{./fig/}}

% Definitions de la page titre
\renewcommand{\maketitle}{%
   \thispagestyle{empty}%
   \pagenumbering{roman}%
   \begin{titlepage}%
      \setcounter{page}{999}%                      % associe le numero "0" a la page titre
      \begin{flushleft}
         \includegraphics[width=12em]{ul_logo}
      \end{flushleft}\par
      \vspace*{\stretch{2}}
      \begin{center}
         \parbox{\textwidth}{\centering\Large\bfseries \theTitreProjet}         \\[5ex]
         \parbox{\textwidth}{\centering\large\bfseries \theTitreRapport}        \\
         \vspace*{\stretch{2}}
         présenté à                    \\[1ex]
         \textbf{\theDestinataire}                 \\
         \vspace*{\stretch{1}}
      \end{center}
      \renewcommand{\arraystretch}{1.5}            % plus d'espace pour les signatures
			\centering
      \begin{tabular}
         {|c|>{\raggedright\arraybackslash}c|}
         \hline
         \multicolumn{1}{|c|}{\emph{matricule}}
         & \multicolumn{1}{c|}{\emph{nom}}
         \\\hline
         \theTableauMembres
      \end{tabular}\par
      \vspace*{\stretch{1}}
      \begin{center}
         {\large Université Laval}             \\
         \theDateRemise
      \end{center}
   \end{titlepage}%
   \clearpage%
   \let\maketitle\relax%
}


% Definitions des fonctions permettant de definir les parametres de la page titre
%
%\newcommand{\theTitreProjet}{\foreignlanguage{french}{Titre du projet}}                  % Titre du projet
%\newcommand{\TitreProjet}[1]{\renewcommand{\theTitreProjet}{\foreignlanguage{french}{#1}}}           % Titre du projet
\def\TitreProjet#1{\def\theTitreProjet{#1}}           % Titre du projet
\def\TitreRapport#1{\def\theTitreRapport{#1}}         % Titre du rapport
\def\Destinataire#1{\def\theDestinataire{#1}}         % Nom(s) du destinataire
\def\NumeroEquipe#1{\def\theNumeroEquipe{#1}}         % Numero de l'equipe
\def\NomEquipe#1{\def\theNomEquipe{#1}}               % Nom de l'equipe
\def\TableauMembres#1{\def\theTableauMembres{#1}}     % Tableau des membres de l'equipe
\def\DateRemise#1{\def\theDateRemise{#1}}             % Date de remise
\def\HistoriqueVersions#1%
   {\def\theHistoriqueVersions{#1}}                   % Historique des versions
 

% Definitions des parametres par defaut de la page titre
\def\theTitreProjet{Titre du projet}                  % Titre du projet
\def\theTitreRapport{Titre du rapport}                % Titre du rapport
\def\theDestinataire{Destinataire}                    % Nom(s) du destinataire
\def\theNumeroEquipe{XX}                              % Numero de l'equipe
\def\theNomEquipe{Nom de l'équipe}                % Nom de l'equipe
\def\theTableauMembres{matricule & nom & \\\hline}    % Tableau des membres de l'equipe
\def\theDateRemise{Date de remise}                    % Date de remise
\def\theHistoriqueVersions%
   {version & date & description \\\hline}            % Historique des versions


% Definition du niveau hierarchique maximum couvert par la table des matieres
\setcounter{tocdepth}{3}            % default = 2

% Definition du niveau hierarchique maximum ayant une numerotation
\setcounter{secnumdepth}{3}         % default = 2

% Definitions des parametres du package hyperref - details propres au rapport
\AtBeginDocument{%
   \hypersetup{pdftitle={\theTitreProjet},%
               pdfsubject={\theTitreRapport},%
               pdfauthor={Équipe \theNumeroEquipe\ - \theNomEquipe}}%
}

% Selection de la langue "babel" par defaut
\AtBeginDocument{%
   \selectlanguage{french}%
}

% Changement des mots utilises par "babel" par defaut (figure et table)
\AtBeginDocument{%
   \renewcommand{\figurename}{\textsc{Figure}}%
   \renewcommand{\tablename}{\textsc{Tableau}}%
}

% Generation de la page titre
\AtBeginDocument{%
   \maketitle
}

% Reinitialisation du compteur "page" (valeur et style)
\AtBeginDocument{%
   \cleardoublepage%
   \setcounter{page}{1}%
   \pagenumbering{arabic}%
}
