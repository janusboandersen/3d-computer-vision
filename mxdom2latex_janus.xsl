<?xml version="1.0" encoding="utf-8"?>

<!--
This is an XSL stylesheet which converts mscript XML files into XSLT.
Use the XSLT command to perform the conversion.

Ned Gulley and Matthew Simoneau, September 2003
Copyright 1984-2013 The MathWorks, Inc.

Modified by Janus Bo Andersen:
September 2019: Structure in report format for AU, add code syntax highlighting
December 2019: Add references system, insert titlepic package, create matlab command
January 2020: Update to E4DSA course

-->

<!DOCTYPE xsl:stylesheet [ <!ENTITY nbsp "&#160;"> ]>
<xsl:stylesheet
  version="1.0"
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  xmlns:escape="http://www.mathworks.com/namespace/latex/escape"
  xmlns:mwsh="http://www.mathworks.com/namespace/mcode/v1/syntaxhighlight.dtd">
  <xsl:output method="text" indent="no"/>

<xsl:template match="mscript">

\documentclass[a4paper]{report}
\usepackage[margin=1in]{geometry}
%\usepackage{array}
%\setlength\extrarowheight{2pt} % or whatever amount is appropriate for tables
\usepackage{polyglossia}
\setdefaultlanguage[variant=british]{english}
\usepackage{titlepic}
\usepackage{graphicx}
\usepackage{xcolor}
\usepackage{mathtools}
\usepackage{amsfonts, amssymb}
\usepackage{float} %ensures that figures / tables can be floated on page
\usepackage{fancyref}
\usepackage[sorting=none,backend=biber]{biblatex}
\usepackage{notoccite}
\addbibresource{../references.bib}

\newcommand{\MATLAB}{\textsc{Matlab}}

\newcommand{\sbul}{\begin{itemize}}
\newcommand{\ebul}{\end{itemize}}

\newcommand{\matr}[1]{{#1}}
\newcommand{\T}{\intercal}
\renewcommand{\vec}[1]{\bar{#1}}            % Regular vector
\newcommand{\ecvec}[1]{\mathbf{#1}}         % Euclidian coordinate vector
\newcommand{\hcvec}[1]{\mathbf{#1}}         % Homogeneous coordinate vector
\newcommand{\ipvec}[1]{\boldsymbol{#1}}     % Image point vector
\DeclareMathOperator{\Rot}{Rot}             % Rotatation transformation
\DeclareMathOperator{\Tr}{trace}            % Trace operator

\newcommand{\E}{\mathrm{E}}
\newcommand{\Var}{\mathrm{Var}}
\newcommand{\Cov}{\mathrm{Cov}}

\usepackage{setspace}
\onehalfspacing % line spacing (is equal to baselinestretch 1.33)

\usepackage[numbered,framed]{matlab-prettifier}
\lstset{
  language           = Matlab,
  style              = Matlab-editor,
  basicstyle         = \mlttfamily,
  escapechar         = `,
  mlshowsectionrules = true
}

%\lstloadlanguages{[ISO]C++,[99]C}


<!-- https://tex.stackexchange.com/questions/283170/using-different-colors-for-different-keywords-in-lstlisting -->
\usepackage{listings}

\definecolor{commentgreen}{RGB}{2,112,10}
\definecolor{eminence}{RGB}{108,48,130}
\definecolor{weborange}{RGB}{255,165,0}
\definecolor{frenchplum}{RGB}{129,20,83}
\definecolor{backgroundColour}{rgb}{0.95,0.95,0.92}

\lstdefinestyle{C++}{
    language=C++,
    escapechar=\%,
    backgroundcolor=\color{backgroundColour},
    commentstyle=\color{commentgreen},
    keywordstyle=\color{eminence},
    stringstyle=\color{red},
    basicstyle=\small\ttfamily,
    emph={int,char,double,float,unsigned,void,bool,uint32_t,uint16_t},
    emphstyle={\color{blue}},
    breakatwhitespace=false,
    breaklines=true,
    captionpos=b,
    keepspaces=true,
    numbers=left,
    numbersep=5pt,
    showspaces=false,
    showstringspaces=false,
    showtabs=false,
    tabsize=2,
    showstringspaces=false
}

\usepackage[binary-units=true]{siunitx}
\sisetup{detect-all}

%\definecolor{vertmatlab}{RGB}{28,160,55}
%\definecolor{mauvematlab}{RGB}{155,71,239}
%\definecolor{fond}{RGB}{246,246,246}
\definecolor{lightgray}{gray}{0.7}

\sloppy
\setlength{\parindent}{0pt}

\usepackage{titlesec}
\titleformat{\chapter}{\huge\bf}{\thechapter.}{20pt}{\huge\bf}
\titleclass{\chapter}{straight}

\usepackage[colorlinks,citecolor=blue,linkcolor=black,urlcolor=blue]{hyperref}

\author{Janus Bo Andersen \thanks{Student id: JA67494. Mail: ja67494@post.au.dk or janus@janusboandersen.dk}}
\titlepic{\includegraphics[width=12cm]{../img/cover.png}}


\begin{document}

\pagenumbering{roman}


    <!-- Determine if the there should be an introduction section. -->
    <xsl:variable name="hasIntro" select="count(cell[@style = 'overview'])"/>
    <xsl:if test = "$hasIntro">
    \title{<xsl:apply-templates select="cell[1]/steptitle"/>}
        <xsl:apply-templates select="cell[1]/text"/>
    </xsl:if>

    \maketitle

\tableofcontents
\newpage

\pagenumbering{arabic}

    <xsl:variable name="body-cells" select="cell[not(@style = 'overview')]"/>

    <!-- Include contents if there are titles for any subsections. -->
<!--    <xsl:if test="count(cell/steptitle[not(@style = 'document')])">
      <xsl:call-template name="contents">
        <xsl:with-param name="body-cells" select="$body-cells"/>
      </xsl:call-template>
    </xsl:if>
-->
    <!-- Loop over each cell -->
    <xsl:for-each select="$body-cells">
        <!-- Title of cell -->
        <xsl:if test="steptitle">
          <xsl:variable name="headinglevel">
            <xsl:choose>
              <xsl:when test="steptitle[@style = 'document']">chapter</xsl:when>
              <xsl:otherwise>section</xsl:otherwise>
            </xsl:choose>
          </xsl:variable>

<!-- \<xsl:value-of select="$headinglevel"/>*{<xsl:apply-templates select="steptitle"/>} -->
\<xsl:value-of select="$headinglevel"/>{<xsl:apply-templates select="steptitle"/>}

        </xsl:if>

        <!-- Contents of each cell -->
        <xsl:apply-templates select="text"/>
        <xsl:apply-templates select="mcode"/>
        <xsl:apply-templates select="mcodeoutput"/>
        <xsl:apply-templates select="img"/>

    </xsl:for-each>


<xsl:if test="copyright">
\begin{par} \footnotesize \color{lightgray} \begin{flushright}
\emph{<xsl:apply-templates select="copyright"/>}
\end{flushright} \color{black} \normalsize \end{par}
</xsl:if>

<!-- Manually insert at right location in doc
\newpage
\printbibliography -->

\end{document}

</xsl:template>



<xsl:template name="contents">
  <xsl:param name="body-cells"/>
\section{Contents}

\begin{itemize}
\setlength{\itemsep}{-1ex}<xsl:for-each select="$body-cells">
      <xsl:if test="./steptitle">
   \item <xsl:apply-templates select="steptitle"/>
      </xsl:if>
    </xsl:for-each>
\end{itemize}
</xsl:template>




<!-- HTML Tags in text sections -->
<xsl:template match="p">\begin{par}
<xsl:apply-templates/>
<xsl:text>
\end{par} <!-- \ vspace{1em} -->
</xsl:text>
</xsl:template>

<xsl:template match="ul">\begin{itemize}
\setlength{\itemsep}{-1ex}
<xsl:apply-templates/>\end{itemize}
</xsl:template>
<xsl:template match="ol">\begin{enumerate}
\setlength{\itemsep}{-1ex}
<xsl:apply-templates/>\end{enumerate}
</xsl:template>
<xsl:template match="li">   \item <xsl:apply-templates/><xsl:text>
</xsl:text></xsl:template>

<xsl:template match="pre">
  <xsl:choose>
    <xsl:when test="@class='error'">
\begin{verbatim}<xsl:value-of select="."/>\end{verbatim}
    </xsl:when>
    <xsl:otherwise>
\begin{verbatim}<xsl:value-of select="."/>\end{verbatim}
    </xsl:otherwise>
  </xsl:choose>
</xsl:template>
<xsl:template match="b">\textbf{<xsl:apply-templates/>}</xsl:template>
<xsl:template match="tt">\texttt{<xsl:apply-templates/>}</xsl:template>
<xsl:template match="i">\textit{<xsl:apply-templates/>}</xsl:template>
<xsl:template match="a">\begin{verbatim}<xsl:value-of select="."/>\end{verbatim}</xsl:template>

<xsl:template match="text()">
  <!-- Escape special characters in text -->
  <xsl:call-template name="replace">
    <xsl:with-param name="string" select="."/>
  </xsl:call-template>
</xsl:template>

<xsl:template match="equation">
<xsl:value-of select="."/>
</xsl:template>

<xsl:template match="latex">
    <xsl:value-of select="@text" disable-output-escaping="yes"/>
</xsl:template>
<xsl:template match="html"/>


<!-- Code input and output -->

<xsl:template match="mcode">
\begin{lstlisting}[language=Matlab, style=Matlab-editor]
<xsl:value-of select="."/>
\end{lstlisting}
</xsl:template>


<xsl:template match="mcodeoutput">
  <xsl:choose>
    <xsl:when test="substring(.,0,8)='&lt;latex&gt;'">
      \end{par}<xsl:value-of select="substring(.,8,string-length(.)-16)" disable-output-escaping="yes"/>\begin{par}
    </xsl:when>
    <xsl:otherwise>
        \color{lightgray} \begin{verbatim}<xsl:value-of select="."/>\end{verbatim} \color{black}
    </xsl:otherwise>
  </xsl:choose>
</xsl:template>


<!-- Figure and model snapshots -->

<xsl:template match="img">
\begin{figure}[H]
\centering
\includegraphics [width=14cm]{<xsl:value-of select="@src"/>}
\caption{}
\end{figure}
</xsl:template>

<!-- Colors for syntax-highlighted input code -->

<xsl:template match="mwsh:code">\begin{verbatim}<xsl:apply-templates/>\end{verbatim}
</xsl:template>
<xsl:template match="mwsh:keywords">
  <span class="keyword"><xsl:value-of select="."/></span>
</xsl:template>
<xsl:template match="mwsh:strings">
  <span class="string"><xsl:value-of select="."/></span>
</xsl:template>
<xsl:template match="mwsh:comments">
  <span class="comment"><xsl:value-of select="."/></span>
</xsl:template>
<xsl:template match="mwsh:unterminated_strings">
  <span class="untermstring"><xsl:value-of select="."/></span>
</xsl:template>
<xsl:template match="mwsh:system_commands">
  <span class="syscmd"><xsl:value-of select="."/></span>
</xsl:template>


<!-- Used to escape special characters in the LaTeX output. -->

<escape:replacements>
  <!-- special TeX characters -->
  <replace><from>$</from><to>\$</to></replace>
  <replace><from>&amp;</from><to>\&amp;</to></replace>
  <replace><from>%</from><to>\%</to></replace>
  <replace><from>#</from><to>\#</to></replace>
  <replace><from>_</from><to>\_</to></replace>
  <replace><from>{</from><to>\{</to></replace>
  <replace><from>}</from><to>\}</to></replace>
  <!-- mainly in code -->
  <replace><from>~</from><to>\ensuremath{\tilde{\;}}</to></replace>
  <replace><from>^</from><to>\^{}</to></replace>
  <replace><from>\</from><to>\ensuremath{\backslash}</to></replace>
  <!-- mainly in math -->
  <replace><from>|</from><to>\ensuremath{|}</to></replace>
  <replace><from>&lt;</from><to>\ensuremath{&lt;}</to></replace>
  <replace><from>&gt;</from><to>\ensuremath{&gt;}</to></replace>
</escape:replacements>

<xsl:variable name="replacements" select="document('')/xsl:stylesheet/escape:replacements/replace"/>

<xsl:template name="replace">
  <xsl:param name="string"/>
  <xsl:param name="next" select="1"/>

  <xsl:variable name="count" select="count($replacements)"/>
  <xsl:variable name="first" select="$replacements[$next]"/>
  <xsl:choose>
    <xsl:when test="$next > $count">
      <xsl:value-of select="$string"/>
    </xsl:when>
    <xsl:when test="contains($string, $first/from)">
      <xsl:call-template name="replace">
        <xsl:with-param name="string"
                        select="substring-before($string, $first/from)"/>
        <xsl:with-param name="next" select="$next+1" />
      </xsl:call-template>
      <xsl:copy-of select="$first/to" />
      <xsl:call-template name="replace">
        <xsl:with-param name="string"
                        select="substring-after($string, $first/from)"/>
        <xsl:with-param name="next" select="$next"/>
      </xsl:call-template>
    </xsl:when>
    <xsl:otherwise>
      <xsl:call-template name="replace">
        <xsl:with-param name="string" select="$string"/>
        <xsl:with-param name="next" select="$next+1"/>
      </xsl:call-template>
    </xsl:otherwise>
  </xsl:choose>
</xsl:template>

</xsl:stylesheet>
