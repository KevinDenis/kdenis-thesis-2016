Week 3
======

Bereikt
-------

-   Een Configuration-Space implementatie in MATLAB.\
    IN: (convex) polygon robot en (convex) polygon obstakel UIT :
    mogelijke botsingsvrije “posen” die de robot kan aannemen. Methode
    toegepast van Computational Geometry: Algorithms and Applications
    (Minkowski Sum en convhull (matlab functie)). *Was zeker geen
    vereiste, maar ik vond het gewoon interessant*.

-   Verschillende manieren Bézier spline te berekenen met als gedachte
    zo snel mogelijke berekening, nodig voor implementatie. *Ook vrij
    vroeg, maar belangrijk in mijn ogen, zodat ik weet op welke
    verschillende manieren zo’n curve berekend kunnen worden en hun
    rekentijd*.

-   Snelle lectuur van discrete padplanners (LaValle).

-   Eerste draft van algoritme doorsturen en bespreken voor de
    implementatie in MATLAB.

Algoritme voor locale padplanner
--------------------------------

### Overzicht

1.  Het algoritme verwacht de configuration space waarin de robot zich
    kan verplaatsen ($x,y,\theta $), zodat deze kan nakijken of het
    gegenereerde paden botsingsvrij is. Er moet ook een subgoal zijn
    waar de robot zich naar toe moet verplaatsen. Eens dat de robot
    dicht genoeg is bij deze subgoal, moet deze naar de volgende gaan.

2.  Huidige positie en oriëntatie van de robot ($x,y,\theta $).

3.  Een cirkel rond de robot legt de maximale afstand die de splines
    mogen afleggen vergelen met de huidige positie.

4.  Enkel een bepaald deel van de cirkel (vet blauw) zal uiteindelijk
    gebruikt mogen worden waar de eindpunten van de splines zich
    zullen bevinden.

5.  Splines worden gegenereerd met als startpunt de positie van de robot
    en zijn oriëntatie. Er zijn dus beperkingen op de plaats van de
    controle punten voor de Bézier curves. Op dit moment word er met
    cubische Bézier curves, deze bestaan uit 4 controle punten. De
    eigenschappen van een Bézier curves zijn dat ze door punt 1 en
    n (=4) gaan, en de oriëntatie (1ste afgeleide) word bepaald door
    punt 2 en n-1 (=3). De volgende parameters zijn vast :

    1.  Plaats van punt 1 op actuele positie van robot.

    2.  Richtingscoëfficiënt tussen punten 1 en 2 (= actuele
        oriëntatie robot).

    3.  Richtingscoëfficiënt tussen punt 3 en 4 moet er voor zorgen dat
        oriëntatie robot op cirkel “meer” wijst naar subgoal als in zijn
        begin oriëntatie.

    4.  Plaats van punt 4 : op een deel van cirkel.

    De volgende parameters zijn vrij :

    1.  Punt 2 op de rechte bepaald door de oriëntatie en beginpunt van
        de robot.

    2.  Punt 3 op de rechte bepaald door de eind oriëntatie en punt 4.

    3.  Punt 4 op specifiek deel van de cirkel.

6.  De berekende Bézier curves in (4) zullen natuurlijk met een bepaalde
    costfunctie (dicht bij user-intention, lage kromming, afstand, etc.)
    met elkaar vergeleken worden. Men zou ook de Bézier cruve kunnen
    genereren tot de subgoal (in het grijs) en daar informatie van
    gebruiken voor het overwegen van welke locale pad het beste is
    (afstand, kromming, etc.).

### Motivatie en specifieke uitleg voor positie controle punten

![\[fig:Algo2\]Uitleg over de plaats van de vrije controle punten. Rood:
robot. Zwart : locale pad gemaakt dankzij een Bézier Curve. Geel :
ligging van controle punt voor deze specifieke Bézier curve.
Stippellijnen duiden aan hoe controle punten 2, 3 en 4 mogelijk van
plaats kunnen veranderen. De term resolutie betekend het aantal
mogelijke liggingen op de
rechte/kromme.](Algoritme_2.pdf){width="40.00000%"}

#### Waarom deze blauwe cirkel ? {#waarom-deze-blauwe-cirkel .unnumbered}

Het idee van de cirkel kwam door de volgende reden : om een Bézier curve
te kunnen maken zijn er controle punten nodig. Om er voor te zorgen dat
men niet in het wilde weg controle punten begin te genereren en om het
accent tot waar de controle punten zich kunnen bevinden, was de cirkel
in mijn ogen een goede oplossing.

#### Hoe kiest men nu deze deel van de cirkel die gebruikt wordt ?

Ik denk dat ik mij hier kan laten inspireren door de Vector Field
Histogram (normal, + en \*) van Borenstein en Koren, deze hebben een
interessante implementatie voor het kiezen van vrije sectoren. Op deze
manier zou ik dus vrije sectoren op de blauwe cirkel rond de robot
kunnen kiezen.

#### Waarom nu die 2de Bézier curve ?

Ik denk dat er uit deze 2de Bézier curve nuttige informatie kan gehaald
worden. Het antwoord in een zekere zin de vraag *“Hoe ver (positie en
orientatie) ben ik van het subgoal”*.

#### En de resolutie ?

Er zijn verschillende parameters die een bepaalde resolutie kunnen
hebben (de plaatsen van punten 2 en 3 op de rechte bepaald door
respectievelijk punten 1 en 4) en de plaats van punt 4 op de cirkel.

Opmerkingen voor volgende week
------------------------------

-   Dynamische obstakel ontwijking literatuur, kijken hoe ik de ideeën
    kan implementeren in deze padplanner.

-   Literatuur doornemen die in de opmerkingen van Meneer
    Demeester staat.

-   Verder werken aan draft algoritme padplanner na feedback.

-   Bijwerken van referentie bestand voor literatuur !!!
