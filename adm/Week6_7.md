Week 6-7
========

Bereikt
-------

-   Lokale padplanner formuleren als een COP, gegeven begin- en
    eindpositie en oriëntatie, voor een Bézierkromme van orde $n-1$. $n$
    is hier het aantal controlepunten. Deze houdt nu rekening met
    één obstakel. Deze moet ook convex zijn. De methode steunt op de
    theorema van de scheidende hypervlakken, uitgelegd in
    @BoydVandenberghe2004.

-   Update doelstellingen masterproef en 4 concrete
    meetbare objectieven.

Overzicht van padplanner {#overzicht-van-padplanner .unnumbered}
------------------------

Illustratie van de padplanner geformuleerd als een COP in MATLAB
\[eq:COP\]. De parameter die wordt geoptimaliseerd is de positie van de
$n$ controlepunten nodig voor het definiëren van de Bézierkromme van
orde $n-1$.

Voorlopig is de obstakel-ontwijker een zeer dure operatie (van 0.2s naar
1.2s uitvoertijd). Dit kom omdat in het optimaliseringsproces op elke
punt van Bézierkromme controleert of deze ver genoeg van het obstakel
is. In @MercyEtAl2016, is er een elegantere methode (en ook sneller)
ontwikkeld. Deze methode is nog niet geïmplementeerd.

![\[fig:sepHyperPlanes\] Tussen twee niet overlappende convexe sets C en
D is er altijd een scheidende hypervlak, bepaald voor $a^Tx=b$. $a$ is
hier de vector bepaald door de twee dichtstbijzijnde punten in C en D.
$b$ de offset van deze vlak. In deze toepassing is C de robot en is
voorlopig tot een cirkel herleid. D is hier een obstakel gevormd door
een convexe polygoon.](sepHyperPlanes.PNG){width="70.00000%"}

-   Randvoorwaarden. *Voorlopig enkel geometrisch*.

    -   Begin- en eindpunt liggen vast \[eq:startPos,eq:endPos\]

    -   Oriëntatie op begin- en eindpunt liggen vast
        \[eq:startOrient,eq:endOrient\]

    -   Maximale toegelaten kromming \[eq:maxCurvature\]

    -   Er moet een hypervlak getekend kunnen worden tussen elk punt van
        de kromme en het obstakel \[eq:obsavoidance\].

    -   Controlepunten moeten binnen een bepaalde grens blijven
        \[eq:boundX,eq:boundY\]

In \[fig:BezierOptObs\] kan men de resultaten van het
optimaliseringsprobleem vinden met en zonder rekening te houden met een
obstakel.

![\[fig:BezierOptObs\]Optimalisatie met en zonder rekening houden van
het obstakel, respectievelijk in het blauw en
rood.](BezierOptObs.eps){width="70.00000%"}

Wat ik heb gelezen
------------------

-   @BoydVandenberghe2004

-   @MercyEtAl2016

Opmerkingen voor volgende week
------------------------------

-   Dynamica toevoegen, dus ook “hoe gaat men het pad volgen”
    (feedforward, feedback, mogelijke inspiratie in @MercyEtAl2016 en
    case-studie *Optimization of Mechatronic Systems*).

-   Van een cirkelvormige geometrie naar een convexe polynoom geometrie
    voor het obstakel-ontwijking.

-   Afspraak maken met promotors en mentors.

-   Gitlab aanvullen met eventueel literatuur, meeting rapport
    en logboek.
