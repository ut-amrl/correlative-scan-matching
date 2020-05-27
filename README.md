# Correlative Scan Matching Implementation

This repository is an implementation of the paper by Edward Olson named [_Real-Time Correlative Scan Matching_](https://april.eecs.umich.edu/media/pdfs/olson2009icra.pdf).
The main feature is the ability to compute the relative transformation between two lidar scans (represented as 2D pointclouds). In the process of finding the most-likely transformation between two scans we also find the PDF of the transformation distribution and the associated uncertainty.
