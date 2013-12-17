cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  librover
  VERSION 0.1
  RELEASE alpha
  SUMMARY "The interface library for the NIFTi rover."
  AUTHOR "BlueBotics"
  CONTACT "support@bluebotics.com"
  HOME "http://www.bluebotics.com"
  LICENSE "GNU Lesser General Public License (LGPL)"
  INSTALL /usr
)

remake_doc(
  html
)

remake_pack_deb()
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  librover
)
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)
