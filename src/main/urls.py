from django.urls import path
from . import views

urlpatterns = [
    path('', views.map_view, name='map'),
    path('tf/', views.marker_data, name='marker_data'),
]