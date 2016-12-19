# Semi-Autonomous Mobile Robot Navigation in Populated Enviroments
Master thesis of Kevin Denis, under supervision of Professor Bruyninckx and Professor Demeester, mentored by Johan Philips. 

## Doelstelling
De doelstelling van deze masterproef is om een lokale padplanner te ontwikkelen die paden genereert in een dynamische omgeving
De padplanner moet de dynamiek van zijn omgeving kunnen schatten om botsingsvrije pad op te kunnen stellen. 
Deze complexe lokale paden bestaan uit derde en vierde orde Bézierkrommes. 
Het voorgesteld algoritme is een uitbreiding van een al bestaande padplanner die enkel circulaire paden genereert.
Deze schiet tekort in sommige situaties, b.v.b. wanneer de rolstoel naast een deur is. 
Dit algoritme heeft als hoofddoel om toegepast te kunnen worden op een semi-autonome rolstoelen, welke bijkomende vereisten met zich meebrengt. 
Er moet niet één mogelijk pad uit dit algoritme komen maar meerdere. 
Dit, omdat deze gegenereerde paden als hypothesen gebruikt moeten worden voor de navigatie-intentie van de gebruiker. 
Het algoritme moet snel werken, (doel : 5 Hz) zodat de gebruiker niet merkt dat er vertraging is tussen zijn commando en het uitvoeren daarvan.
Uiteindelijk moet het algoritme op een multi-resolutie niveau werken, 
deze moet op korte afstand de invloed van het draaien van het zwenkwiel in rekening brengen
en de dynamische randvoorwaarden van van de rolstoel zelf (maximale versnelling/vertraging, maximale kromming van pad, enz.).

## Objectieven (NL)
1. Ontwerpen van een lokale padplanner gebaseerd op Bézierkromme, geformuleerd als een COP.
2. Deze padplanner moet dynamische obstakels kunnen ontwijken.
3. Deze padplanner moet de dynamische beperkingen van de rolstoel in rekening brengen.
4. Deze padplanner moet snel genoeg uitgevoerd worden zodat de gebruiken geen last heeft van vertraging.


## Objectives (EN)
1. Develop a local path planner based on splines formulated as a constraint optimization problem
2. This path planner should be able to include dynamic obstacle avoidance
3. Take into account the dynamics of the wheelchair
4. Fast enough so that the user doesn't feel the latency (goal : 5 Hz)

## Git organization

### src (Source code)
Here is all the source code used for this thesis.

### adm (Administration)
Here, all the administrative mather for this thesis is stored (e.g. logs, meeting rapport, etc.).

### ppr (Paper)
Here, the source code for the paper and pdf's will be available.

### prs	(Presentation)
Here, the source code for the presentation (mid-term and final) and pdf's will be available.