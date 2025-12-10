{{ name | escape | underline}}

.. currentmodule:: {{ module }}

.. autoclass:: {{ objname }}
   :members:
   :undoc-members:
   :inherited-members:
   :special-members: __call__

   {% block methods %}
   {% if methods %}
   {% endif %}
   {% endblock %}
