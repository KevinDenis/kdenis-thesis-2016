Week 8-9
========

Bereikt
-------

-   Lokale padplanner formuleren als een COP, gegeven begin positie en
    oriëntatie, voor een Bézierkromme van orde $n-1$. $n$ is hier het
    aantal controlepunten. Deze houdt nu rekening met één obstakel. Deze
    moet ook convex zijn. De methode steunt op de theorema van de
    scheidende hypervlakken en een lineaire clasificatie methode,
    uitgelegd in @BoydVandenberghe2004. Belangrijke toevoegingen
    vergeleken met vorige log :

    -   Eind positie en oriëntatie zijn geen randvoorwaarden meer. Deze
        wordt wel gebruikt in de doelfunctie
        (zie \[eq:ObjectiveFunction\])

    -   Aanpassing van \[eq:ObjectiveFunction\], de verschillende termen
        zijn nu geschaald hun maximale waarde, dit maakt het wegen van
        de afzonderlijke termen makkelijker, zoals in
        @DemeesterEtAl2003.

    -   Na een gesprek met de Assistenten van Numerische Optimalisatie
        maak ik terug gebruik van Casadi. Deze vonden dat de fmincon
        (niet-lineaire optimalisatie) functie van MATLAB
        onbetrouwbaar was. De formulering van mijn COP van MATLAB
        ondersteunde solver naar de Casadi solver heeft veel
        tijd genomen.

    -   Obstakel ontwijker steunt nu op een conservatievere maar
        efficiëntere methode.

-   Gitlab repo heeft nu een betere structuur. Daar zullen nu ook
    consequent updates staan van mijn logboek.

Overzicht van padplanner
------------------------

Padplanner geformuleerd als een COP in \[eq:COP\]. De parameter die
wordt geoptimaliseerd is de positie van de $n$ controlepunten nodig voor
het definiëren van de Bézierkromme van orde $n-1$. De doelfunctie komt
deels uit @BruyninckxReynaerts1997 [@DemeesterEtAl2003].

Voorlopig is de obstakel-ontwijker zeer conservatief. Er moet gelden dat
het convexe omhulsel, gevormd door de $n$ controlepunten[^1] en de
obstakel gescheiden kan worden door een hypervlak (zoals in
@MercyEtAl2016), dit zorgt er echter wel voor een snellere rekentijd.

-   Randvoorwaarden. *Voorlopig enkel geometrisch*.

    -   Begin positie en oriëntatie liggen vast
        \[eq:startPos,eq:startOrient\]

    -   Maximale toegelaten kromming \[eq:maxCurvature\]

    -   Er moet een hypervlak getekend kunnen worden tussen elk punt van
        de kromme en het obstakel \[eq:SepHyperPath,eq:SepHyperObs\]
        (uit @BoydVandenberghe2004).

    -   Controlepunten moeten binnen een bepaalde grens blijven
        \[eq:boundX,eq:boundY\]

In \[fig:BezierOptObs\] kan men de resultaten van het
optimaliseringsprobleem vinden.


Wat ik heb gelezen
------------------

-   @MercyEtAl2016

-   @BoydVandenberghe2004

-   @DemeesterEtAl2003

-   @NashKoenig2013

-   @PivtoraikoKelly2012

Opmerkingen voor volgende week
------------------------------

-   Obstakel ontwijker minder conservatief maken. Mogelijke oplossing:
    gebruik maken van de Casteljau algoritme (1 Bézierkromme in 2
    splitsen), en dan het convex omhulsel van deze twee
    curves gebruiken. Andere methode beschreven in @Kamermans2013 het
    convex omhulsel kleiner maken (“tight boxing of Bézier curves”).

-   Afspraak voorbereiden van donderdag 1 december met promotors
    en mentors.

-   Gitlab aanvullen met eventueel literatuur en meeting rapport.

[^1]: Voor een Bézierkromme geld dat deze zich altijd binnen het convexe
    vlak bevindt, beschreven door zijn controlepunten.
