Week 1 - 2 {#week-1---2 .unnumbered}
==========

Doel {#doel .unnumbered}
----

-   Opmerkingen van Meneer Bruyninckx toevoegen aan huidig visie
    doelstelling masterproef

-   Een Configuration-Space implementatie in MATLAB uitwerken.

-   Verder uitwerking van het stuksgewijs toevoegen van Bézier Curves,
    en graad van continuiteit garanderen.

Bereikt {#bereikt .unnumbered}
-------

-   Opmerkingen van Meneer Bruyninckx toevoegen aan huidig visie
    doelstelling masterproef

-   Verder uitwerking van het stuksgewijs toevoegen van Bézier Curves.

Cubic Spline Interpolation {#cubic-spline-interpolation .unnumbered}
--------------------------

### Pro’s {#pros .unnumbered}

-   Polynoom gaat door alle gegeven punten.

-   Oriëntatie kan op een indirecte manier gegeven worden, door de
    waarde van de helling van de begin en eindpunt op te geve

-   Toevoegen van punt zal enkel voor een lokale verandering zorgen
    (groot voordeel tegenover bvb Lagrange Interpolatie)

### Con’s {#cons .unnumbered}

-   Werkt enkel met stijgende waarde van x.

-   Spline kan niet evenwijdig met x-as beginnen en evenwijdig met y as
    eindigen

Mogelijke oplossing voor deze problemen : Door spline in meerdere delen
te berekenen en het lokaal assenstelsel te roteren kan dit probleem
opgelost worden

Cubic Spline Approximation {#cubic-spline-approximation .unnumbered}
--------------------------

### Pro’s {#pros-1 .unnumbered}

-   Op eerste zicht niet logisch, maar door grote gewichten op begin en
    eindpunt te zetten zorgt men voor paden die door begin en eindpunt
    gaan

-   Pad zal kleinere krommingen bevatten, comfortabeler voor de
    passagier

-   Door met de interpoleer gewichten variëren creëert men licht
    verschillende paden.

### Con’s {#cons-1 .unnumbered}

-   Zelfde opmerkingen als bij Cubic Spline Interpolation

Bézier curves {#bézier-curves .unnumbered}
-------------

### Pro’s {#pros-2 .unnumbered}

Cubic Bézier Curve lijkt een ideale oplossing, omdat :

-   Helling curve (dus oriëntatie robot) wordt bepaald door punt 2
    en n-1.

-   Bézier curves kunnen ook aan elkaar bevestigd worden. Voorbeeld
    algoritme: De Casteljau Construction (“Bézier Spline”).

-   Wel ervoor zorgen dat G-2 en C-2 continue is (zeker wanneer curve
    uit meerdere Bézier curves opbouw)

### OPM {#opm .unnumbered}

-   Indien ik C2 continu Bézier curves aan elkaar toe wil voegen, en nog
    steeds vrij wil zijn voor positie en oriëntatie van begin en
    eindpunt, MOET ik 5de orde (4de graad) Bézier curves gebruiken.

Maar, moet dit echt? Zou een controller niet makkelijk deze niet
continue versnelling op kunnen vangen?

Controle punten {#controle-punten .unnumbered}
---------------

Tot nu toe heb ik niet het probleem bekeken van waar ik de punten krijg,
die ik gebruik als controle punt voor de splines. Deze zouden, onder
andere kunnen komen van een globale pad planner. Kleine variaties op
deze punten en orientaties zullen er voor zorgen dat ik meerdere paden
kan maken\
Een andere mogelijkheid zou zijn dat ik enkel de huidige positie en
oriëntatie, doel en de free-space heb, en hiermee paden moet genereren.
Dit laat veel meer vrijheid toe voor het genereren van paden

Concrete vragen {#concrete-vragen .unnumbered}
---------------

1.  Mijn dilemma is het volgende : moet ik mij enkel in Bézier curves
    verdiepen, of proberen een twee oplossing te implementeren, dus
    zowel Cubic Spline Approximation en Bézier curves ?

2.  Is C-2 continu eigenlijk een vereiste? Of is G-2 continu genoeg? Dit
    zou overeenkomen als een “lichte” storing voor de controller;
    versnelling in dezelfde richting maar niet zelfde amplitude

3.  Meneer Bruyninckx, zou u de term “dichtheid van paden” kunnen
    uitleggen?

Opmerkingen voor volgende week {#opmerkingen-voor-volgende-week .unnumbered}
------------------------------

-   Een concrete methode uitwerken voor het genereren van punten als
    input voor de locale pad planner.

-   Invloed stad van castor wielen op pad

-   Soccer robots van Eindhoven University of Technology bekijken.

-   Planning Algorithms van Steven M. LaValle doornemen

-   Doornemen masterproef van Karel Belaen (gebruik van splines voor het
    afvlakken van paden)
