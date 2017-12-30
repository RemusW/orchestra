defmodule InteropProxy.Requests do
  @moduledoc """
  Handles making requests to the AUVSI SUAS Interoperability Server.

  After logging in, the cookie must be passed to the server after
  subsequent requests.
  """

  @json       {"Content-Type", "application/json"}
  @png        {"Content-Type", "image/png"}
  @urlencoded {"Content-Type", "application/x-www-form-urlencoded"}

  @doc """
  Log in to the server and get a cookie.
  """
  def login(url, username, password) do
    body = _get_urlencoded %{username: username, password: password}

    "http://#{url}/api/login"
    |> HTTPoison.post(body, [@urlencoded])
    |> _handle_login
  end

  @doc """
  Get the full list of missions from the server.
  """
  def get_missions(url, cookie) do
    "http://#{url}/api/missions"
    |> _cookie_get(cookie)
    |> _handle_json
  end

  @doc """
  Get a specific mission from the server.
  """
  def get_mission(url, cookie, id) when is_integer(id) do
    "http://#{url}/api/missions/#{id}"
    |> _cookie_get(cookie)
    |> _handle_json
  end

  @doc """
  Get the stationary and movie obstacles from the server.
  """
  def get_obstacles(url, cookie) do
    "http://#{url}/api/obstacles"
    |> _cookie_get(cookie)
    |> _handle_json
  end

  @doc """
  Post UAS telemetry to the server.
  """
  def post_telemetry(url, cookie, telem) when is_map(telem) do
    body = _get_urlencoded(telem)

    "http://#{url}/api/telemetry"
    |> _cookie_post(cookie, body, [@urlencoded])
    |> _handle_text("UAS Telemetry Successfully Posted.")
  end

  @doc """
  Post a new ODLC to the server.
  """
  def post_odlc(url, cookie, odlc) when is_map(odlc) do
    body = _get_json(odlc)

    "http://#{url}/api/odlcs"
    |> _cookie_post(cookie, body)
    |> _handle_json
  end

  @doc """
  Get all ODLCs from the server.
  """
  def get_odlcs(url, cookie) do
    "http://#{url}/api/odlcs"
    |> _cookie_get(cookie)
    |> _handle_json
  end

  @doc """
  Get a ODLC from the server.
  """
  def get_odlc(url, cookie, id) when is_integer(id) do
    "http://#{url}/api/odlcs/#{id}"
    |> _cookie_get(cookie)
    |> _handle_json
  end

  @doc """
  Update an ODLC on the server.
  """
  def put_odlc(url, cookie, id, odlc) when is_integer(id) and is_map(odlc) do
    body = _get_json(odlc)

    "http://#{url}/api/odlcs/#{id}"
    |> _cookie_put(cookie, body)
    |> _handle_json
  end

  @doc """
  Delete an ODLC from the server.
  """
  def delete_odlc(url, cookie, id) when is_integer(id) do
    "http://#{url}/api/odlcs/#{id}"
    |> _cookie_delete(cookie)
    |> _handle_text("Object deleted.")
  end

  @doc """
  Get an ODLC's image from the server.
  """
  def get_odlc_image(url, cookie, id) when is_integer(id) do
    "http://#{url}/api/odlcs/#{id}/image"
    |> _cookie_get(cookie)
    |> _handle_get_image
  end

  @doc """
  Post an ODLC's image from the server.
  """
  def post_odlc_image(url, cookie, id, image)
  when is_integer(id) and is_binary(image) do
    "http://#{url}/api/odlcs/#{id}/image"
    |> _cookie_post(cookie, image, [@png])
    |> _handle_text("Image uploaded.")
  end

  @doc """
  Update an ODLC's image from the server.

  This works the same as posting a new image.
  """
  def put_odlc_image(url, cookie, id, image),
    do: post_odlc_image(url, cookie, id, image)

  @doc """
  Delete an ODLC's image from the server.
  """
  def delete_odlc_image(url, cookie, id) when is_integer(id) do
    "http://#{url}/api/odlcs/#{id}"
    |> _cookie_delete(cookie)
    |> _handle_text("Image deleted.")
  end

  # Making sure we have a succesful login, and getting the cookie
  # from the response.
  defp _handle_login({:ok, %{status_code: 200, body: body, headers: h}}) do
    headers = Enum.into h, %{}

    cookie = headers["Set-Cookie"]
    |> String.split(";")
    |> List.first

    {:ok, body, cookie}
  end

  # Making sure we're getting a png back.
  defp _handle_get_image({:ok, %{status_code: 200, body: body, headers: h}})
  when is_binary(body) do
    headers = Enum.into h, %{}

    # We'll have a match error if it's not a png.
    @png = {"Content-Type", headers["Content-Type"]}

    {:ok, body}
  end

  # Handling generic JSON responses.
  defp _handle_json({:ok, %{status_code: code, body: body}})
  when code in [200, 201] do
    {:ok, Poison.Parser.parse!(body)}
  end

  # Handling and verifying a generic text response.
  defp _handle_text({:ok, %{status_code: code, body: body}}, expected)
  when code in [200, 201] and body === expected do
    {:ok, body}
  end

  # Making a urlencoded message from a map.
  defp _get_urlencoded(body) when is_map(body) do
    body
    |> Enum.map(fn {k, v} -> "#{Atom.to_string(k)}=#{v}" end)
    |> Enum.join("&")
  end

  # Making a json encoded message from a map.
  defp _get_json(body) when is_map(body),
    do: Poison.encode!(body)

  # Send a POST request with a cookie (default is json body).
  defp _cookie_post(url, cookie, body, headers \\ [@json]),
    do: HTTPoison.post(url, body, headers ++ [{"Cookie", cookie}])

  # Send a GET request with a cookie.
  defp _cookie_get(url, cookie, headers \\ []),
    do: HTTPoison.get(url, headers ++ [{"Cookie", cookie}])

  # Send a PUT request with a cookie (default is json body).
  defp _cookie_put(url, cookie, body, headers \\ [@json]),
    do: HTTPoison.put(url, body, headers ++ [{"Cookie", cookie}])

  # Send a DELETE request with a cookie.
  defp _cookie_delete(url, cookie, headers \\ []),
    do: HTTPoison.delete(url, headers ++ [{"Cookie", cookie}])
end
