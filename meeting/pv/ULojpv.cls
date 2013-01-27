%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%                                                             %%%%%%%%%%%%%
%%%%%%%%%%%%%   Classe de document "ULojpv"                              %%%%%%%%%%%%%
%%%%%%%%%%%%%   par Pierre Tremblay, Universite Laval                     %%%%%%%%%%%%%
%%%%%%%%%%%%%   2008/01/06 - version 1.1                                  %%%%%%%%%%%%%
%%%%%%%%%%%%%                                                             %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------------------------------------------------------------------
%----------------------------------- identification -----------------------------------
%--------------------------------------------------------------------------------------
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{ULojpv}%
              [2008/01/06 v1.1a Classe d'ordre du jour et de proces-verbal %
               - Universite Laval]
%--------------------------------------------------------------------------------------
%------------------------------------ initial code ------------------------------------
%--------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------
%------------------------------- declaration of options -------------------------------
%--------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------
%-------------------------------- execution of options --------------------------------
%--------------------------------------------------------------------------------------
\ProcessOptions \relax
%--------------------------------------------------------------------------------------
%----------------------------------- package loading ----------------------------------
%--------------------------------------------------------------------------------------
\LoadClassWithOptions{report}
%
\RequirePackage{lmodern}
\RequirePackage[T1]{fontenc}
\RequirePackage[french]{babel}
\RequirePackage{tabularx}
%--------------------------------------------------------------------------------------
%-------------------------------------- main code -------------------------------------
%--------------------------------------------------------------------------------------
%**************************************************************************************
% Redefinitions :
%    - du format d'affichage des compteurs de la liste "enumerate"
%    - du format d'affichage des étiquettes de la liste "enumerate"
%
\renewcommand{\theenumi}{\arabic{enumi}}
\renewcommand{\labelenumi}{\textbf{\theenumi.}}
\renewcommand{\theenumii}{\arabic{enumii}}
\renewcommand{\labelenumii}{\theenumi.\theenumii.}
\renewcommand{\theenumiii}{\arabic{enumiii}}
\renewcommand{\labelenumiii}{\theenumi.\theenumii.\theenumiii.}
%**************************************************************************************
%**************************************************************************************
% Definitions de l'en-tête
%
\newcommand{\entete}{%
   \thispagestyle{empty}%
   \begin{center}
      \Large\scshape%
      \theCours                                    \\
      Équipe \theNumeroEquipe\ --- \theNomEquipe   \\[1ex]
      \theObjet
   \end{center}
   \medskip
   \noindent
   \begin{tabularx}{\textwidth}{@{}r@{}>{\raggedright\hspace{0pt}\arraybackslash}Xr@{}l@{}}
      \hline
      \scshape Sujet~:~    & \theSujetRencontre
      & \scshape Date~:~   & \theDateRencontre
      \\
      \scshape Local~:~    & \theLocalRencontre
      & \scshape Heure~:~  & \theHeureRencontre
      \\\hline
   \end{tabularx}
   \medskip
   \pagestyle{myheadings}
   \markright{Équipe \theNumeroEquipe\ --- \theObjet\ (\theDateRencontre)}
}
%**************************************************************************************
%**************************************************************************************
% Definitions des fonctions permettant de definir les parametres de la page titre
%
\def\Cours#1{\def\theCours{#1}}                       % Nom du cours
\def\NumeroEquipe#1{\def\theNumeroEquipe{#1}}         % Numero de l'equipe
\def\NomEquipe#1{\def\theNomEquipe{#1}}               % Nom de l'equipe
\def\Objet#1{\def\theObjet{#1}}                       % Nom du document
\def\SujetRencontre#1{\def\theSujetRencontre{#1}}     % Sujet de la rencontre
\def\DateRencontre#1{\def\theDateRencontre{#1}}       % Date de la rencontre
\def\LocalRencontre#1{\def\theLocalRencontre{#1}}     % Local de la rencontre
\def\HeureRencontre#1{\def\theHeureRencontre{#1}}     % Heure de la rencontre
%**************************************************************************************
%**************************************************************************************
% Definitions des parametres par defaut de la page titre
%
\def\theCours{Numéro et nom du cours}                 % Numero et nom du cours
\def\theNumeroEquipe{XX}                              % Numero de l'equipe
\def\theNomEquipe{Nom de l'équipe}                    % Nom de l'equipe
\def\theObjet{Nom du document}                        % Nom du document
\def\theSujetRencontre{Le sujet}                      % Sujet de la rencontre
\def\theDateRencontre{La date}                        % Date de la rencontre
\def\theLocalRencontre{Le local}                      % Local de la rencontre
\def\theHeureRencontre{L'heure}                       % Heure de la rencontre
%**************************************************************************************
