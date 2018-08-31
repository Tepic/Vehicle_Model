Trenutna verzija u publish-u se sastoji od 2 glavna programa i nekoliko pomocnih funkcija.

* DoubleLaneChange - predstavlja predefinisani test ISO standarda. Nedostatak ovog testa je sto se odrzava konstantna brzina vozila. Potrebno je ubaciti model trenja cime bi se na taj nacin uticalo na brzinu vozila.
* userDefinedTest - predstavlja kod za razlicita upravljanja vozilom - FREE TEST

Dodatne funkcije:
- doubleLaneChange - generise cunjeve i kapije za test na mestima koji su zadati kao ulazni parametri funkcije
- plotData - iscrtava sve parametre simulacije - poslednji ulazni argument funkcije mora biti {'DLC','etc'}. DLC je Double-Lane-Change, dok je etc - sve ostalo (free test)
- v07 - je glavni program od kojeg je sve poteklo
- wheel_move je glavna simulaciona funkicja koja zapravo vrsi pomeraj vozila u prostoru kroz vreme na osnovu ulaznih argumenata (pogledati kod)

*** generateOrientation - koristi se za generisanje reference upravljanja - polozaja vozila tokom vremena. Vraca vektor orijentacije vozila u odredjenom trenutku vremena i vremenski vektor. Poslednji argument je {'ramp','step'} - tip generisanja upravljacke reference

Dodatni fajlovi su tire_Avon - nosi podatke o pneumatiku koriscenom na FSRA16, wheel_speeds - brzina pojedinacnog tocka generisan iz neke od prethodnih simulacija (trenutno u ovom kodu neupotrebljiv, jer se ne vrsi jos uvek upravljanje brzinom tockova) - za ovako nesto je potrebno dodati momentne jednacine uticaja opterecenja i brzine svakog tocka na ceo sistem. Sistem je ionako previse nelinearan, stoga je ovo trenutno bilo previse za brzu implementaciju. To je jedan od narednih koraka unapredjenja.
