# How to Install

## Building OpenCV

From a WSL prompt:

```
cd ~
```

I then followed the OpenCV installation instructions at this link: https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html, specifically building with opencv_contrib.

## Python Setup

I use [`mamba` installed via `mambaforge`](https://mamba.readthedocs.io/en/latest/installation.html) as my preferred version of `conda`. See the `.yml` files for the list of packages. These files were created with:

```
mamba env export --with-history > environment.yml
```

In theory you should be able to load them with:

```
mamba env create -f environment.yml
```

though I have not tested this extensively yet.