An [EPICS](http://www.aps.anl.gov/epics/) module that supports Industry Pack digital I/O modules.

Devices supported in ipUnidig include:
* From [SBS](http://www.sbs.com)
  * The IP-UD family of digital I/O modules. This family was formerly called the IP-Unidig.
    There are many models in this family, and most are supported. However, the only
    model that I have actually tested is the IP-UD-I, (formerly IP-Unidig-I) which is
    a 24-channel Input/Output model with interrupt support.
  * IP-OPTOIO-8 eight-bit isolated digital I/O.

* From [Systran](http://www.systran.com)
  * DIO316I, 48-bit digital I/O module. The software currently only supports 24 bits
    on this device, and does not support the interrupt feature.

The support is written as an asynPortDriver using the the [EPICS asyn module](https://github.com/epics-modules/asyn)

Additional information:
* [Home page](http://cars.uchicago.edu/software/epics/ipUnidig.html)
* [Documentation](http://cars.uchicago.edu/software/epics/ipUnidigDoc.html)
* [Release notes](http://cars.uchicago.edu/software/epics/ipUnidigReleaseNotes.html)
