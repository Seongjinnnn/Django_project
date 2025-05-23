from django.urls import path
from . import views

urlpatterns = [
    path('', views.map_view, name='map'),
    path('vehicle/', views.vehicle_info, name='vehicle_info'),  # 변경된 부분
]
