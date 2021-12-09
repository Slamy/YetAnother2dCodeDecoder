# Yet another 2D code decoder written in C++

## Why does this exist?

For quite some time, I wondered how exactly the Reed Solomon forward error correction worked.
It was magical to me but by reading short articles about it, I never grasped the theory behind it.
So someday I decided to give it a shot and read some white papers about it and tried to create a Reed Solomon decoder and encoder from scratch.

In the end, it was so much fun, that I've decided to actually use it for something. As I didn't intend to develop a CD drive all by myself I went for the route of decoding a Qr code as it is quite modern and lots of example data is around the internet to check my algorithms against.

This project is intended for pure academic purposes.
The algorithms are not tuned for speed but instead readability. At least I've tried to achieve that.

## Features

* Reed Solomon Encoder and Decoder
    * Implemented as template class to allow usage with various finite fields
* Finite field GF(2^8) with Qr code and data matrix irreducible polynomial
* Utility classes for managing polynomials and matrices
* Qr Code Decoder
    * Can rectify perspective deforms
    * Can recitfy slight nonlinearities
    * Doesn't support all sizes
    * Only one alignment pattern supported
* Data Matrix Decoder
    * Supports only ECC200 codes
    * Only small sizes supported
* Unit Test included for easy verification
* Uses OpenCV to process image data
    * Both Decoders make use of the Countour system to detect the code as from polygons.

