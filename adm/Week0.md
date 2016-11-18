Week 0 {#week-0 .unnumbered}
======

Hierin is het werk gemaakt in de weken voor het begin van de
academiejaar gebundeld.

Doel {#doel .unnumbered}
----

-   Concreet doel voor thesis tegen eind september.

-   Literatuur studie over het implementeren van splines in
    pad planning.

-   Leren werken met gitlab.

Bereikt {#bereikt .unnumbered}
-------

-   Concreet doel voor thesis tegen eind september.\
    \
    Eerste visie is neergeschreven. Deze zal tijdens week 1
    verfijnt worden.

-   Literatuur studie over het implementeren van splines in pad
    planning.\
    \
    Methodes voor het afvlakken van paden die op een andere manier
    berekend zijn.

    -   Bezier Curves : curve gaat door begin en eindpunt. Raaklijn
        begin en eindpunt hangt af van punt 2 en n-1. Dit is handig,
        want ik ben volledig vrij in het bepalen van het begin en eind
        oriëntatie van de curve. Begin = huidige pose, eind =
        gewenste pose.

    -   Splines : elke controle punt heeft hetzelfde gewicht. Graad
        bepaald continuïteit van de punten. Er zou makkelijk gekozen
        kunnen worden voor het gebruik van cubische splines. Deze zorgen
        ervoor dat de functie in elk punt continue is tot de 2de
        afgeleide (pad is C2 continu). Er is natuurlijk een duidelijk
        verschil tussen heb gebruik maken van spline methodes of af te
        vlakken en om te interpoleren.

    -   NURBS : cfr. splines maar controle punt heeft een vrij te kiezen
        gewicht

Opmerkingen voor volgende week {#opmerkingen-voor-volgende-week .unnumbered}
------------------------------

-   Benchmark maken om meerdere methodes (Bezier Curves, Splines
    en NURBS) uit te testen, zodat werking, parameters, vrijheid en
    rekentijd gekend zijn.

-   Leren werken met gitlab.

Wat ik heb gelezen {#wat-ik-heb-gelezen .unnumbered}
------------------

-   @DemeesterEtAl2012

-   @ConnorsElkaim2007a

-   @ConnorsElkaim2007

-   @ChoiEtAl2008

-   @ChoiEtAl2010

-   @ElbanhawiEtAl2015

-   @ErenEtAl1999

-   @WalambeEtAl2016
