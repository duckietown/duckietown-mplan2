# coding=utf-8 
__version__ = '0.1.0'

import logging

logging.basicConfig()
logger = logging.getLogger('dt-mplan')
logger.setLevel(logging.DEBUG)

logger.info('duckietown_mplan %s' % __version__)

from .algo import *

