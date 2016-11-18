Week 4-5
========

Bereikt
-------

-   Lokale padplanner formuleren als een COP, gegeven begin- en
    eindpositie en oriëntatie, voor een Bézierkromme van orde $n-1$. $n$
    is hier het aantal controlepunten.

-   Eerste stappen in gitlab.

Overzicht van padplanner
------------------------

Illustratie van de padplanner geformuleerd als een COP in MATLAB, met
CASADI @Andersson2013 en OPTISTACK (dit is een front-end/wrapper van
CASADI voor MATLAB).

-   Parameter\
    De parameter die wordt geoptimaliseerd is de positie van de $n$
    controlepunten nodig voor het definiëren van de Bézierkromme van
    orde $n-1$. $$z = [x_{1} , y_{1}, \dots, x_{n} , y_{n} ]^T$$
    $$B(t) = BézierCurve(x,y,t)$$

    &\
    & t =0:t:1,   &

-   Kostfunctie (minimaliseren) (uit @BruyninckxReynaerts1997 [^1])\
    $$f(z) = \int_0^1{\kappa(t)^2dt}+\alpha\int_0^1{s(t)dt}$$

-   Randvoorwaarden. *Voorlopig enkel geometrisch*.

    -   Begin- en eindpunt liggen vast

        \[x\_1,y\_1\] &= \[x1,y1\]\
        \[x\_<span>n</span>,y\_<span>n</span>\] &= \[xn,yn\]

    -   Oriëntatie op begin- en eindpunt liggen vast

        y\_2 &= (\_1)(x\_2-x\_1)\
        y\_<span>end-1</span> &=
        (\_<span>end</span>)(x\_<span>end</span>-x\_<span>end-1</span>)

    -   Maximale toegelaten kromming
        $$\kappa(t) \leqslant \kappa_{max}$$

    -   controlepunten $2:n-1$ moeten binnen de rechthoek blijven
        gedefinieerd door punten $1$ en $n$.
        $$[x_1,y_1]  <=[x_{2:n-1},y_{2:n-1}]<= [x_{n},y_{n}]$$

In \[fig:BezierOptAlpha\] en \[fig:BezierOptN\] kan men figuren vinden
van de optimalisatie resultaten met toenemende waarde van $\alpha$ en
$n$ respectievelijk.

Wat ik heb gelezen
------------------

-   @BruyninckxReynaerts1997

-   @MercyEtAl2016

-   @WalambeEtAl2016

-   @Kamermans2013

-   @Sederberg2016

Opmerkingen voor volgende week
------------------------------

-   Obstakel ontwijking toevoegen @MercyEtAl2016 en referentie
    hierin (Hypervlak).

-   Gitlab aanvullen met eventueel literatuur, meeting rapport
    en logboek.

-   Update doelstellingen masterproef en 4 concrete
    meetbare objectieven.

[^1]: hier wordt er geïntegreerd over de dimensieloze parameter $t$.
    Niet te verwarren met de tijd. In @BruyninckxReynaerts1997 wordt er
    over $ds$ geïntegreerd, dit heb ik voorlopig nog niet
    geïmplementeerd.
