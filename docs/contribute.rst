How to Contribute
=================

Introduction
------------

This wiki website is hosted on `readthedocs <https://readthedocs.org/>`_,
meaning that contributors only need to write the docs and push to GitHub repository,
and `readthedocs <https://readthedocs.org/>`_
will manage the domain and server backend for this website.

The contributors can use either
`reStructuredText <https://en.wikipedia.org/wiki/ReStructuredText>`_
(``.rst`` file, preferred) or
`markdown <https://en.wikipedia.org/wiki/Markdown>`_ (``.md`` file)
syntax to write the documentation source code.
It will be compiled into HTML webpages using Sphinx theme.

To write and compile the docs locally,
it is preferred to have an Ubuntu 16.04 (or above) operating system and
follow the instructions below.

Installation
------------

- If you don't have ``pip`` or ``pip3`` installed, you can install it by

  .. code:: bash

    sudo apt-get install python-pip
    sudo apt-get install python3-pip

- According to `Sphinx Starter Tutorial
  <https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html>`_
  and `Theme Installation Tutorial <https://sphinx-rtd-theme.readthedocs.io/en/latest/installing.html>`_,
  you need to have the following packages installed.

  .. code:: bash

    pip3 install sphinx
    pip3 install recommonmark
    pip3 install sphinx_rtd_theme

- Note that this is different from the tutorial on Sphinx website,
  we replace ``pip`` (which will cause error) with ``pip3`` here.

Make Changes
------------

- You can git clone this repository and work on it locally.

  .. code:: bash

    git clone https://github.com/UCR-Robotics/UCR-Robotics.git

- To make changes to the existing webpage,
  you just need to edit the corresponding source file using correct syntax.
  (Note: you can only use reStructuredText syntax in files ending with ``.rst``
  and use markdown syntax in files ending with ``.md``. You cannot mix these two.)

- To add a new file, please create a new file ``filename.rst`` or ``filename.md``
  under docs folder, and then add ``filename`` (without extension)
  to the toctree in ``index.rst`` file.

- If you are ready, you can compile the HTML documentation locally.

  .. code:: bash

    cd UCR-Robotics/docs
    make html

- The resulting HTML webpages are in ``UCR-Robotics/docs/_build/html`` folder.

- Since you can preview the result of your webpage,
  you can actually edit your files locally and rebuild until you like what you see,
  then commit the changes and push to the GitHub repository.

- For `ARCS Lab <https://sites.google.com/view/arcs-lab>`_ group members,
  you can refresh the webpages after you push to GitHub repository.
  It usually takes less than a minute for readthedocs server
  to build the HTML webpages.

- For others who do not have push access to the repository,
  please open a pull request and we will respond as soon as possible.
  We appreciate your valuable contribution.

Tutorials on reStructuredText
-----------------------------

``reStructuredText`` is a markup structure (similar to ``markdown``) that supports rich features
to help you build up your own HTML documentations in a fast and easy way.

- `Quick Starter Tutorial
  <http://docutils.sourceforge.net/docs/user/rst/quickref.html>`_
  (read this first)

- `reStructuredText Markup Specification
  <http://docutils.sourceforge.net/docs/ref/rst/restructuredtext.html>`_
  (more details)

- `About Code Blocks
  <https://sublime-and-sphinx-guide.readthedocs.io/en/latest/code_blocks.html>`_

- `About Directives
  <http://docutils.sourceforge.net/docs/ref/rst/directives.html>`_
