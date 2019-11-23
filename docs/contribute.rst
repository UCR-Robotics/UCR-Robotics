How to Contribute
=================


Installation
------------

- According to `Sphinx Starter Tutorial 
  <https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html>`_
  and `Theme Installation Tutorial <https://sphinx-rtd-theme.readthedocs.io/en/latest/installing.html>`_,
  you need to have the following installed.

  .. code:: bash

    pip3 install sphinx
    pip3 install recommonmark
    pip3 install sphinx_rtd_theme

- Note that this is different from the tutorial website, 
  because ``pip`` will cause error and only ``pip3`` works well.

- If you don't have ``pip`` or ``pip3`` installed, you can install it by

  .. code:: bash

    sudo apt-get install python-pip
    sudo apt-get install python3-pip

- With the above dependencies installed, you can compile the HTML documentation locally.

  .. code:: bash

    cd UCR-Robotics/docs
    make html

- The resulting HTML webpages are in ``UCR-Robotics/docs/_build/html`` folder.

- Since you can preview the result of your webpage,
  you can actually edit your files and rebuild until you like what you see, 
  then commit your changes and push to your public repository. 


Tutorials on reStructuredText
-----------------------------

``reStructuredText`` is a markup structure (similar to ``markdown``) that supports rich features
to help you build up your own HTML documentations in a fast and easy way.

- `Quick Starter Tutorial 
  <http://docutils.sourceforge.net/docs/user/rst/quickref.html>`_ (recommended reading first)

- `reStructuredText Markup Specification 
  <http://docutils.sourceforge.net/docs/ref/rst/restructuredtext.html>`_ (a syntax lookup book)

- `About Code Blocks 
  <https://sublime-and-sphinx-guide.readthedocs.io/en/latest/code_blocks.html>`_

- `About Directives 
  <http://docutils.sourceforge.net/docs/ref/rst/directives.html>`_

